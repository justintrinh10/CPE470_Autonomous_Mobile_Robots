import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Float32
import numpy as np
from .point import Point
from .path import Path

class MoveRobotFollowPath(Node):
    def __init__(self):
        super().__init__("move_robot_follow_path")
        self.publisher_ = self.create_publisher(Bool, "move_robot_follow_path_complete", 10)

        self.publisher_move_distance_ = self.create_publisher(Float32, "move_robot_distance", 10)
        self.subscriber_move_distance_complete_ = self.create_subscription(
            Bool,
            "move_robot_distance_complete",
            self.listener_callback_move_distance_complete,
            10,
        )

        self.publisher_rotate_angle_ = self.create_publisher(Float32, "rotate_robot_angle", 10)
        self.subscriber_rotate_angle_complete_ = self.create_subscription(
            Bool,
            "rotate_robot_angle_complete",
            self.listener_callback_rotate_angle_complete,
            10,
        )

        self.subscriber_move_robot_follow_path = self.create_subscription(
            String,
            "move_robot_follow_path",
            self.listener_callback_move_robot_follow_path,
            10,
        )

        self.state = "IDLE"
        self.current_index = 0
        self.num_segments = 0
        self.path = None

    def listener_callback_move_robot_follow_path(self, msg):
        self.path = self.convert_msg_to_path(msg)
        self.current_index = 0
        self.num_segments = len(self.path.distances)
        self.state = "IDLE"
        
        if self.num_segments < 1:
            self.get_logger().info("Path has less than 2 points. Nothing to do.")
            complete_msg = Bool()
            complete_msg.data = True
            self.publisher_.publish(complete_msg)
            return
        
        angle_command = self.path.angle_changes[self.current_index]
        angle_msg = Float32()
        angle_msg.data = float(angle_command)
        self.publisher_rotate_angle_.publish(angle_msg)
        self.state = "ROTATING"

    def listener_callback_move_distance_complete(self, msg):
        if self.state != "MOVING":
            return
        self.current_index += 1
        if self.current_index < self.num_segments:
            self.get_logger().info(f"Move Distance Complete {self.current_index}")

            angle_command = self.path.angle_changes[self.current_index]
            angle_msg = Float32()
            angle_msg.data = float(angle_command)
            self.publisher_rotate_angle_.publish(angle_msg)
            self.state = "ROTATING"

        else:
            self.get_logger().info("Move Robot Follow Path Complete")

            complete_msg = Bool()
            complete_msg.data = True
            self.publisher_.publish(complete_msg)
            self.state = "IDLE"
            return

    def listener_callback_rotate_angle_complete(self, msg):
        if self.state != "ROTATING":
            return
        if msg.data:
            self.get_logger().info(f"Rotation Complete {self.current_index}")

            self.state = "MOVING"
            distance_msg = Float32()
            distance_msg.data = float(self.path.distances[self.current_index])
            self.publisher_move_distance_.publish(distance_msg)

    def convert_msg_to_path(self, msg):
        path_points = msg.data.strip().split("\n")
        points = []
        for point_str in path_points:
            x_str, y_str = point_str.split(",")
            x = float(x_str)
            y = float(y_str)
            points.append(Point(x, y))
        path = Path(points)
        return path

def main(args=None):
    rclpy.init(args=args)
    move_robot_follow_path = MoveRobotFollowPath()
    rclpy.spin(move_robot_follow_path)
    move_robot_follow_path.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()