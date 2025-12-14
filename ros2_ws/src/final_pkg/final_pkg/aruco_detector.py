import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv2 import aruco
from geometry_msgs.msg import Twist
from checkpoint_interfaces.msg import ParametersToTarget
from std_msgs.msg import Bool
import math

CAMERA_TO_ROBOT_CENTER = 0.12  # meters

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        self.measurement_pub = self.create_publisher(
            ParametersToTarget, 'aruco_measurement', 10
        )
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.localized_sub = self.create_subscription(
            Bool, 'localization_complete', self.stop_scanning, 10
        )

        # Camera
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        # ArUco
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters_create()

        # Calibration
        self.camera_matrix = np.array([
            [821.993, 0, 330.489],
            [0, 821.993, 248.997],
            [0, 0, 1]
        ])
        self.dist_coeffs = np.array([[-0.018522, 1.03979, 0, 0, -3.3171]])
        self.marker_length = 0.05

        self.scanning = True
        self.timer = self.create_timer(0.1, self.process_frame)

        self.start_rotation()

    def start_rotation(self):
        cmd = Twist()
        cmd.angular.z = 0.3
        self.cmd_pub.publish(cmd)
        self.get_logger().info("Rotating to scan for ArUco markers")

    def stop_scanning(self, msg):
        if msg.data:
            self.scanning = False
            self.cmd_pub.publish(Twist())
            self.get_logger().info("Stopping rotation — localization complete")

    def process_frame(self):
        if not self.scanning:
            return

        ret, frame = self.cap.read()
        if not ret:
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.parameters
        )

        if ids is None:
            return

        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
            corners, self.marker_length,
            self.camera_matrix, self.dist_coeffs
        )

        for i, marker_id in enumerate(ids.flatten()):
            tvec = tvecs[i][0]
            x, _, z = tvec

            distance = math.sqrt(x**2 + z**2) + CAMERA_TO_ROBOT_CENTER
            bearing = math.degrees(math.atan2(x, z))

            msg = ParametersToTarget()
            msg.marker_id = int(marker_id)
            msg.distance_seperation = distance
            msg.alignment_error = bearing
            self.measurement_pub.publish(msg)

            self.get_logger().info(
                f"Marker {marker_id}: d={distance:.2f}m, θ={bearing:.1f}°"
            )

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main():
    rclpy.init()
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()