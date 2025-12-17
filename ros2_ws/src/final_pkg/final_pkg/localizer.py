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
        if self.localized or len(self.measurements) < 3:
            return

        # Extract marker IDs and distances
        ids = list(self.measurements.keys())
        m1, m2, m3 = ids[0], ids[1], ids[2]

        x1, y1 = MARKER_MAP[m1]
        x2, y2 = MARKER_MAP[m2]
        x3, y3 = MARKER_MAP[m3]

        d1 = self.measurements[m1]
        d2 = self.measurements[m2]
        d3 = self.measurements[m3]

        # Solve linear system from two circle differences
        # (x-x1)^2 + (y-y1)^2 = d1^2
        # (x-x2)^2 + (y-y2)^2 = d2^2
        # Subtract to get linear equation:
        # 2(x2-x1)x + 2(y2-y1)y = x2^2 - x1^2 + y2^2 - y1^2 + d1^2 - d2^2
        A = np.array([
            [2*(x2 - x1), 2*(y2 - y1)],
            [2*(x3 - x1), 2*(y3 - y1)]
        ])
        b = np.array([
            [x2**2 - x1**2 + y2**2 - y1**2 + d1**2 - d2**2],
            [x3**2 - x1**2 + y3**2 - y1**2 + d1**2 - d3**2]
        ])

        # Solve for x, y
        pos, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
        x, y = pos.flatten()

        # Check bounds and publish
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
                f"LOCALIZED at x={x:.3f}, y={y:.3f} using markers {ids}"
            )


def main():
    rclpy.init()
    node = Localizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
