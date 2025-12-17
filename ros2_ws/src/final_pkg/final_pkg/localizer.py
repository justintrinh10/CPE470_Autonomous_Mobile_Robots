import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import String, Bool
from checkpoint_interfaces.msg import ParametersToTarget
from .marker_map import MARKER_MAP

class Localizer(Node):
    def __init__(self):
        super().__init__('localizer')

        # Subscribe to ArUco measurements
        self.sub = self.create_subscription(
            ParametersToTarget,
            'aruco_measurement',
            self.measurement_cb,
            10
        )

        # Publishers
        self.position_pub = self.create_publisher(String, 'robot_position', 10)
        self.done_pub = self.create_publisher(Bool, 'localization_complete', 10)

        self.measurements = {}  # Store marker measurements
        self.localized = False

        # Timer to attempt localization every 0.5s
        self.timer = self.create_timer(0.5, self.try_localize)

    def measurement_cb(self, msg):
        if self.localized:
            return

        if msg.marker_id in MARKER_MAP:
            self.measurements[msg.marker_id] = msg.distance_seperation

    def try_localize(self):
        if self.localized or len(self.measurements) < 2:
            return

        A = []
        b = []

        for mid, d in self.measurements.items():
            xm, ym = MARKER_MAP[mid]
            A.append([2 * xm, 2 * ym])
            b.append([xm**2 + ym**2 - d**2])

        A = np.array(A)
        b = np.array(b)

        # Solve linear system using least squares
        pos, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
        x, y = pos.flatten()  # <-- fix: convert from array([[x],[y]]) to scalars

        # Check if position is within bounds
        if 0 <= x <= 1.15 and 0 <= y <= 0.93:
            self.localized = True

            # Publish position
            msg = String()
            msg.data = f"{x:.3f},{y:.3f}"
            self.position_pub.publish(msg)

            # Publish done signal
            done = Bool()
            done.data = True
            self.done_pub.publish(done)

            self.get_logger().info(
                f"LOCALIZED at x={x:.3f}, y={y:.3f} using markers {list(self.measurements.keys())}"
            )


def main():
    rclpy.init()
    node = Localizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
