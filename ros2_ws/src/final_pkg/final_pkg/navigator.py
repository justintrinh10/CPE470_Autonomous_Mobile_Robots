import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
import math
import time
class Navigator(Node):
    def __init__(self):
        super().__init__('navigator')
        # Default goal (overwritten by start_move_robot_to_point)
        self.goal_x = 0.0
        self.goal_y = 0.0
        # Subscriptions
        self.position_sub = self.create_subscription(
            String,
            'robot_position',
            self.position_cb,
            10
        )
        self.localized_sub = self.create_subscription(
            Bool,
            'localization_complete',
            self.localized_cb,
            10
        )
        self.start_sub = self.create_subscription(
            String,
            'start_move_robot_to_point',
            self.start_move_robot_callback,
            10
        )
        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.done_pub = self.create_publisher(
            Bool, 'move_robot_to_point_complete', 10
        )
        # State
        self.position = None
        self.localized = False
        self.goal_received = False
        self.arrived = False
        self.wait_start = None
        # Control loop
        self.timer = self.create_timer(0.1, self.navigate)
    # ---------------- Callbacks ---------------- #
    def position_cb(self, msg):
        x, y = msg.data.split(',')
        self.position = (float(x), float(y))
    def localized_cb(self, msg):
        if msg.data:
            self.localized = True
            self.get_logger().info("Navigator: Localization complete")
    def start_move_robot_callback(self, msg):
        x, y = msg.data.strip().split(',')
        self.goal_x = float(x)
        self.goal_y = float(y)
        self.arrived = False
        self.wait_start = None
        self.goal_received = True
        self.get_logger().info(
            f"Navigator: New goal received x={self.goal_x:.3f}, y={self.goal_y:.3f}"
        )
    # ---------------- Navigation Logic ---------------- #
    def navigate(self):
        if not self.localized or not self.goal_received or self.position is None:
            return
        if self.arrived:
            return
        x, y = self.position
        dx = self.goal_x - x
        dy = self.goal_y - y
        dist = math.hypot(dx, dy)
        # ----- Goal reached ----- #
        if dist < 0.05:
            if self.wait_start is None:
                self.cmd_pub.publish(Twist())
                self.wait_start = time.time()
                self.get_logger().info("Reached Point P — waiting 3 seconds")
            elif time.time() - self.wait_start >= 3.0:
                self.arrived = True
                self.cmd_pub.publish(Twist())
                msg = Bool()
                msg.data = True
                self.done_pub.publish(msg)
                self.get_logger().info("Navigator: Goal complete")
            return
        # ----- Move toward goal ----- #
        angle_to_goal = math.atan2(dy, dx)
        cmd = Twist()
        cmd.linear.x = 0.15
        cmd.angular.z = 0.5 * angle_to_goal
        self.cmd_pub.publish(cmd)
def main():
    rclpy.init()
    node = Navigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()