import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from checkpoint_interfaces.msg import ParametersToTarget
from cv_bridge import CvBridge
import cv2
import numpy as np
from cv2 import aruco
from geometry_msgs.msg import Twist

angular_velocity_val = 0.005

camera_to_robot_center = 0.12 # meters

class ArucoPoseNode(Node):
    def __init__(self):
        super().__init__('aruco_pose_node')
        self.camera_fov = 60  # degrees

        self.publisher_ = self.create_publisher(ParametersToTarget, "aruco_pose_parameters", 10)

        self.publisher_robot_rotate = self.create_publisher(Twist, '/cmd_vel', 10)


        # Camera setup
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        # ArUco marker setup
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters_create()

        #Camera calibration parameters with our values
        self.camera_matrix = np.array([
            [821.993, 0, 330.489],
            [0, 821.993, 248.997],
            [0, 0, 1]
        ])
        self.dist_coeffs = np.array([[-0.018522, 1.03979, 0, 0, -3.3171, 0, 0, 0]])
        self.marker_length = 0.05  # meters

        # Timer to process frames (~10 FPS)
        self.timer = self.create_timer(0.1, self.process_frame)

    def aruco_display(self, corners, ids, image):
        if len(corners) > 0:
            ids = ids.flatten()
            for (markerCorner, markerID) in zip(corners, ids):
                # Corner order: top-left, top-right, bottom-right, bottom-left
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners

                # Convert to integer
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))

                # Compute center
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)

        else:
            cX, cY = image.shape[1] // 2, image.shape[0] // 2

        return image, (cX, cY)


    def process_frame(self):
        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().warn('Failed to capture frame')
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        frame, (cX, cY) = self.aruco_display(corners, ids, frame)

        if ids is None:
            self.get_logger().error("No ArUco detected")
            command = Twist()
            command.linear.x = 0.0
            command.linear.y = 0.0
            command.linear.z = 0.0
            command.angular.x = 0.0
            command.angular.y = 0.0
            command.angular.z = angular_velocity_val
            self.publisher_robot_rotate.publish(command)
            return
        elif len(ids) > 1:
            self.get_logger().error("More than one ArUco detected")
            return
        
        img_center_x = frame.shape[1] // 2
        horiz_dist_ci_cx = cX - img_center_x

        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, self.marker_length, self.camera_matrix, self.dist_coeffs
        )

        # Extract the vector correctly: (N, 1, 3) → (3,)
        rvec = rvecs[0][0]
        tvec = tvecs[0][0]

        alignment_error = horiz_dist_ci_cx / frame.shape[1] * self.camera_fov
        distance_seperation = self.find_distance_seperation(tvec) + camera_to_robot_center
        msg = ParametersToTarget()
        msg.alignment_error = alignment_error
        msg.distance_seperation = distance_seperation
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published ArUco Pose Parameters: Alignment Error = {alignment_error:.2f} degrees, Distance Seperation = {distance_seperation:.2f} meters")

    def find_distance_seperation(self, tvec):
        tvec = tvec.flatten()
        x, y, z = tvec
        return np.hypot(x, z)

    def __del__(self):
        self.cap.release()
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    aruco_detector = ArucoPoseNode()

    try:
        rclpy.spin(aruco_detector)
    except KeyboardInterrupt:
        pass
    finally:
        aruco_detector.cap.release()
        cv2.destroyAllWindows()
        aruco_detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
