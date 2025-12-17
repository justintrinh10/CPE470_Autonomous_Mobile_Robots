import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy
import math
import time

class Navigator(Node):
    def __init__(self):
        super().__init__('navigator')

        # Goal
        self.goal_x = 0.0
        self.goal_y = 0.0

        # State
        self.position = None
        self.yaw = None
        self.goal_received = False
        self.arrived = False
        self.wait_start = None

        # Subscriptions
        self.position_sub = self.create_subscription(
            String, 'robot_position', self.position_cb, 10
        )
        self.start_sub = self.create_subscription(
            String, 'start_move_robot_to_point', self.start_move_robot_callback, 10
        )

        # /odom subscription with QoS matching publisher
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_cb, odom_qos
        )

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.done_pub = self.create_publisher(
            Bool, 'move_robot_to_point_complete', 10
        )

        # Timer
        self.timer = self.create_timer(0.1, self.navigate)

    # ------------- Callbacks ---------------- #
    def position_cb(self, msg):
        try:
            x, y = msg.data.split(',')
            self.position = (float(x), float(y))
        except:
            self.get_logger().warn("Navigator: Bad robot_position format")

    def start_move_robot_callback(self, msg):
        try:
            x, y = msg.data.strip().split(',')
            self.goal_x = float(x)
            self.goal_y = float(y)
            self.goal_received = True
            self.arrived = False
            self.wait_start = None

            self.get_logger().info(
                f"Navigator: Goal received ({self.goal_x}, {self.goal_y})"
            )
        except:
            self.get_logger().warn("Navigator: Bad goal format")

    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    # ------------- Navigation Loop ---------------- #
    def navigate(self):

        if not self.goal_received or self.position is None or self.yaw is None:
            return

        if self.arrived:
            return

        x, y = self.position
        dx = self.goal_x - x
        dy = self.goal_y - y
        dist = math.hypot(dx, dy)

        cmd = Twist()

        # ---- Arrived ---- #
        if dist < 0.12:  # 12 cm tolerance
            if self.wait_start is None:
                self.cmd_pub.publish(Twist())
                self.wait_start = time.time()
                self.get_logger().info("Reached goal — waiting 3 secs")
            elif time.time() - self.wait_start >= 3.0:
                self.arrived = True
                self.cmd_pub.publish(Twist())
                done = Bool(); done.data = True
                self.done_pub.publish(done)
                self.get_logger().info("Navigator: Goal complete")
            return

        # ---- Move toward goal ---- #
        angle_to_goal = math.atan2(dy, dx)
        angle_error = math.atan2(
            math.sin(angle_to_goal - self.yaw),
            math.cos(angle_to_goal - self.yaw)
        )

        # Angular velocity
        P_ANG = 2.0
        ang = P_ANG * angle_error
        ang = max(-1.5, min(1.5, ang))  # clamp for safety
        cmd.angular.z = ang

        # Forward motion only if roughly aligned
        if abs(angle_error) < 0.3:
            cmd.linear.x = 0.15

        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = Navigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
