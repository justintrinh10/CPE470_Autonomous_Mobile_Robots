import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
import math
import time

class Navigator(Node):
    def __init__(self):
        super().__init__('navigator')

        self.declare_parameter('point_p_x', 0.59)
        self.declare_parameter('point_p_y', 0.46)

        self.goal_x = self.get_parameter('point_p_x').value
        self.goal_y = self.get_parameter('point_p_y').value

        self.position_sub = self.create_subscription(
            String, 'robot_position', self.position_cb, 10
        )
        self.localized_sub = self.create_subscription(
            Bool, 'localization_complete', self.localized_cb, 10
        )

        self.subscriber_start_move_robot_to_point = self.create_subscription(
            String,
            'start_move_robot_to_point',
            self.start_move_robot_callback,
            10
        )
        self.publisher_move_robot_to_point_complete = self.create_publisher(Bool, 'move_robot_to_point_complete', 10)
        self.recieved_goal = False

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.position = None
        self.localized = False
        self.arrived = False
        self.wait_start = None

        self.timer = self.create_timer(0.1, self.navigate)

    def position_cb(self, msg):
        x, y = msg.data.split(',')
        self.position = (float(x), float(y))

    def localized_cb(self, msg):
        if msg.data:
            self.localized = True
            self.get_logger().info("Navigation started")

    def navigate(self):
        if not self.localized or self.position is None or self.arrived or not self.recieved_goal:
            return

        x, y = self.position
        dx = self.goal_x - x
        dy = self.goal_y - y
        dist = math.hypot(dx, dy)

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
                self.publisher_move_robot_to_point_complete.publish(msg)
                self.get_logger().info("Done.")
            return

        angle = math.atan2(dy, dx)

        cmd = Twist()
        cmd.angular.z = 0.5 * angle
        cmd.linear.x = 0.15 if abs(angle) < 0.3 else 0.0
        self.cmd_pub.publish(cmd)

    def start_move_robot_callback(self, msg):
        x, y = msg.data.strip().split(',')
        self.goal_x = float(x)
        self.goal_y = float(y)
        self.arrived = False
        self.wait_start = None
        self.recieved_goal = True
        self.get_logger().info(f"Received new goal: x={self.goal_x}, y={self.goal_y}")

def main():
    rclpy.init()
    node = Navigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
