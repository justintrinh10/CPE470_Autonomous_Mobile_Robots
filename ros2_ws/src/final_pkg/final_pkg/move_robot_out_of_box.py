import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Bool
import numpy as np

class MoveRobotOutOfBox(Node):
    def __init__(self):
        super().__init__("move_robot_out_of_box")
        self.publisher_ = self.create_publisher(Bool, "move_robot_out_of_box", 10)

        self.publisher_move_distance_ = self.create_publisher(String, "move_robot_distance", 10)
        self.subscriber_move_distance_complete_ = self.create_subscription(
            Bool,
            "move_robot_distance_complete",
            self.listener_callback_move_distance_complete,
            10,
        )

        self.publisher_rotate_angle_ = self.create_publisher(String, "rotate_robot_angle", 10)
        self.subscriber_rotate_angle_complete_ = self.create_subscription(
            Bool,
            "rotate_robot_angle_complete",
            self.listener_callback_rotate_angle_complete,
            10,
        )

        self.subscriber_move_robot_out_of_box_ = self.create_subscription(
            Path_Msg,
            "move_robot_out_of_box",
            self.listener_callback_move_robot_out_of_box,
            10,
        )






