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
        self.lidar_running = False

        self.publisher_start_process_lidar = self.create_publisher(String, 'start_process_lidar', 10)
        self.subscriber_process_lidar_complete = self.create_subscription(
            String,
            'process_lidar_complete',
            self.process_lidar_complete_callback,
            10
        )
        self.path = ""
        self.processing_lidar = False

        self.publisher_move_robot_follow_path = self.create_publisher(String, 'move_robot_follow_path', 10)
        self.subscriber_move_robot_follow_path_complete = self.create_subscription(
            Bool,
            'move_robot_follow_path_complete',
            self.move_robot_follow_path_complete_callback,
            10
        )
        self.moving_robot_follow_path = False

    def start_move_callback(self, msg):
        lidar_msg = Int32()
        lidar_msg.data = self.num_points
        self.publisher_start_lidar.publish(lidar_msg)
        self.lidar_running = True

        while self.lidar_running:
            rclpy.spin_once(self)
        
        process_lidar_msg = String()
        process_lidar_msg.data = self.points
        self.publisher_start_process_lidar.publish(process_lidar_msg)
        self.processing_lidar = True

        while self.processing_lidar:
            rclpy.spin_once(self)

        move_robot_follow_path_msg = String()
        move_robot_follow_path_msg.data = self.path
        self.publisher_move_robot_follow_path.publish(move_robot_follow_path_msg)
        self.moving_robot_follow_path = True

        while self.moving_robot_follow_path:
            rclpy.spin_once(self)
        
        complete_msg = Bool()
        complete_msg.data = True
        self.publisher_move_robot_outside_box_complete.publish(complete_msg)
        self.get_logger().info("Move Robot Outside Box Complete")

    def lidar_complete_callback(self, msg):
        self.points = msg.data
        self.lidar_running = False
        self.get_logger().info("Lidar Data Received")
    
    def process_lidar_complete_callback(self, msg):
        self.path = msg.data
        self.processing_lidar = False
        self.get_logger().info("Processed Lidar Data Received")
    
    def move_robot_follow_path_complete_callback(self, msg):
        self.moving_robot_follow_path = False
        self.get_logger().info("Move Robot Follow Path Complete Received")

def main(args=None):
    rclpy.init(args=args)
    move_robot_outside_box = MoveRobotOutsideBox()
    rclpy.spin(move_robot_outside_box)
    move_robot_outside_box.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


