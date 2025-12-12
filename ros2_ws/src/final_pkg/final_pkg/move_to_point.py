import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Bool
import numpy as np
import matplotlib.pyplot as plt

class MoveToPoint(Node):
    def __init__(self):
        super().__init__("move_to_point")
        self.subscriber_ = self.create_subscription(
            String,
            "pointInput",
            self.listener_callback,
            10,
        )
        self.publisher_ = self.create_publisher(String, "movementCommands", 10)

    def listener_callback(self, msg):
        x_str, y_str = msg.data.split(",")
        x = float(x_str)
        y = float(y_str)
        self.get_logger().info(f"Received Target Point: ({x}, {y})")
        command_msg = String()
        command_msg.data = f"MOVE_TO {x} {y}"
        self.publisher_.publish(command_msg)
        self.get_logger().info(f"Published Movement Command to ({x}, {y})")