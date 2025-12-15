import serial
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Bool
import numpy as np

class UserInterface(Node):
    def __init__(self):
        super().__init__("user_interface")

        self.publisher_ = self.create_publisher(String, "user_interface_complete", 10)

        self.subscriber_ = self.create_subscription(
            Bool,
            "start_user_interface",
            self.listener_callback,
            10,
        )
    
    def listener_callback(self, msg):
        self.get_logger().info("Enter desired point. (meters)")
        self.get_logger().info("X: ")
        x = float(input())
        self.get_logger().info("Y: ")
        y = float(input())
        point_msg = String()
        point_msg.data = f"{x},{y}"
        self.publisher_.publish(point_msg)
        self.get_logger().info(f"Published Point: ({x}, {y})")

def main(args=None):
    rclpy.init(args=args)
    user_interface = UserInterface()
    rclpy.spin(user_interface)
    user_interface.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()