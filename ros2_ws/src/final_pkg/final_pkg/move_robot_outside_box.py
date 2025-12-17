import rclpy
from rclpy.node import Node
from checkpoint_interfaces.msg import ParametersToTarget
import numpy as np
from geometry_msgs.msg import Twist
import math
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import String

class MoveRobotOutsideBox(Node):
    def __init__(self):
        super().__init__('move_robot_outside_box')

        self.state = "INIT"

        self.publisher_move_robot_outside_box_complete = self.create_publisher(Bool, 'move_robot_outside_box_complete', 10)
        self.subscriber_start_move_robot_outside_box = self.create_subscription(
            Bool,
            'start_move_robot_outside_box',
            self.start_move_callback,
            10
        )

        self.publisher_start_lidar = self.create_publisher(Int32, 'start_lidar', 10)
        self.subscriber_lidar_complete = self.create_subscription(
            String,
            'lidar_complete',
            self.lidar_complete_callback,
            10
        )
        self.num_points = 500
        self.points = ""

        self.publisher_start_process_lidar = self.create_publisher(String, 'start_process_lidar', 10)
        self.subscriber_process_lidar_complete = self.create_subscription(
            String,
            'process_lidar_complete',
            self.process_lidar_complete_callback,
            10
        )
        self.path = ""

        self.publisher_move_robot_follow_path = self.create_publisher(String, 'move_robot_follow_path', 10)
        self.subscriber_move_robot_follow_path_complete = self.create_subscription(
            Bool,
            'move_robot_follow_path_complete',
            self.move_robot_follow_path_complete_callback,
            10
        )

    def start_move_callback(self, msg):
        lidar_msg = Int32()
        lidar_msg.data = self.num_points
        self.publisher_start_lidar.publish(lidar_msg)
        self.state = "LIDAR_RUNNING"

    def lidar_complete_callback(self, msg):
        if self.state != "LIDAR_RUNNING":
            return
        self.points = msg.data
        self.get_logger().info("Lidar Data Received")

        process_lidar_msg = String()
        process_lidar_msg.data = self.points
        self.publisher_start_process_lidar.publish(process_lidar_msg)
        self.state = "PROCESSING_LIDAR"
    
    def process_lidar_complete_callback(self, msg):
        if self.state != "PROCESSING_LIDAR":
            return
        self.path = msg.data
        self.get_logger().info("Processed Lidar Data Received")

        move_robot_follow_path_msg = String()
        move_robot_follow_path_msg.data = self.path
        self.publisher_move_robot_follow_path.publish(move_robot_follow_path_msg)
        self.state = "MOVING_ROBOT_FOLLOW_PATH"
    
    def move_robot_follow_path_complete_callback(self, msg):
        if self.state != "MOVING_ROBOT_FOLLOW_PATH":
            return
        self.get_logger().info("Move Robot Follow Path Complete Received")

        complete_msg = Bool()
        complete_msg.data = True
        self.publisher_move_robot_outside_box_complete.publish(complete_msg)
        self.get_logger().info("Move Robot Outside Box Complete")
        self.state = "COMPLETE"

def main(args=None):
    rclpy.init(args=args)
    move_robot_outside_box = MoveRobotOutsideBox()
    rclpy.spin(move_robot_outside_box)
    move_robot_outside_box.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()