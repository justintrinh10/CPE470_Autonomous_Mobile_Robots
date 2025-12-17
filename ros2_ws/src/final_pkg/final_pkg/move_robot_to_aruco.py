import rclpy
from rclpy.node import Node
from checkpoint_interfaces.msg import ParametersToTarget
import numpy as np
from geometry_msgs.msg import Twist
import math
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from std_msgs.msg import Float32

alignment_threshold = 1.5  # degrees

class MoveRobotToAruco(Node):
    def __init__(self):
        super().__init__('move_robot_to_aruco')
        self.state = "IDLE"

        self.subscription_aruco_pose_parameter = self.create_subscription(
            ParametersToTarget,
            'aruco_pose_parameters',
            self.listener_callback,
            10
        )
        
        self.publisher_start_aruco_detection = self.create_publisher(Int32, 'start_aruco_detection', 10)

        self.subscription_start_move_to_aruco = self.create_subscription(
            Int32,
            'start_move_robot_to_aruco',
            self.start_move_callback,
            10
        )

        self.publisher_stop_aruco_detection = self.create_publisher(Bool, 'stop_aruco_detection', 10)

        self.publisher_rotate_robot_to_aruco = self.create_publisher(Bool, 'start_rotate_robot', 10)

        self.subscriber_rotate_complete = self.create_subscription(
            Bool,
            'rotation_complete',
            self.rotate_complete_callback,
            10
        )

        self.publisher_move_robot_distance = self.create_publisher(Float32, 'move_robot_distance', 10)
        self.subscriber_move_distance_complete = self.create_subscription(
            Bool,
            'move_robot_distance_complete',
            self.move_distance_complete_callback,
            10
        )

        self.publisher_move_robot_to_aruco_complete = self.create_publisher(Bool, 'move_robot_to_aruco_complete', 10)

        self.Wanted_ID = 1
        self.distance_seperation = 0.0

    def start_move_callback(self, msg):
        self.Wanted_ID = msg.data
        start_msg = Int32()
        start_msg.data = self.Wanted_ID
        self.publisher_start_aruco_detection.publish(start_msg)
        self.get_logger().info(f"Started Move Robot to Aruco with ID: {self.Wanted_ID}")

        start_msg = Bool()
        start_msg.data = True
        self.publisher_rotate_robot_to_aruco.publish(start_msg)
        self.Rotating = True

        while self.Rotating:
            rclpy.spin_once(self)
        
        stop_msg = Bool()
        stop_msg.data = True
        self.publisher_stop_aruco_detection.publish(stop_msg)
        self.get_logger().info("Stopped Aruco Detection after Rotation.")

        move_msg = Float32()
        move_msg.data = self.distance_seperation
        self.publisher_move_robot_distance.publish(move_msg)
        self.Moving = True

        while self.Moving:
            rclpy.spin_once(self)
        
        complete_msg = Bool()
        complete_msg.data = True
        self.publisher_move_robot_to_aruco_complete.publish(complete_msg)
        self.get_logger().info("Move Robot to Aruco Complete.")
        self.Running = False
    
    def rotate_complete_callback(self, msg):
        self.get_logger().info("Rotation to Aruco complete.")
        self.Rotating = False
    
    def move_distance_complete_callback(self, msg):
        self.get_logger().info("Movement to Aruco complete.")
        self.Moving = False
    
    def listener_callback(self, msg):
        self.distance_seperation = msg.distance_seperation


def main(args=None):
    rclpy.init(args=args)
    move_robot_to_aruco = MoveRobotToAruco()
    rclpy.spin(move_robot_to_aruco)
    move_robot_to_aruco.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        
