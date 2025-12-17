import rclpy
from rclpy.node import Node
from checkpoint_interfaces.msg import ParametersToTarget
import numpy as np
from geometry_msgs.msg import Twist
import math
from std_msgs.msg import Bool, Float32, Int32

alignment_threshold = 3.0  # degrees

class RotateRobot2(Node):
    def __init__(self):
        super().__init__('rotate_robot2')
        self.subscription = self.create_subscription(
            ParametersToTarget,
            'aruco_pose_parameters',
            self.listener_callback,
            10
        )
        self.subscription_start_move_robot_to_aruco = self.create_subscription(
            Int32,
            'start_move_robot_to_aruco',
            self.start_move_robot_to_aruco_callback,
            10
        )
        self.running = False

        self.publisher_stop_aruco_pose_parameters = self.create_publisher(Bool, 'stop_aruco_pose_parameters', 10)
        self.publisher_move_robot_distance = self.create_publisher(Float32, 'move_robot_distance', 10)

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.publisher_stop_ = self.create_publisher(Bool, 'rotation_complete', 10)
        self.angular_velocity = 0.005  # radians per second
        self.counter = 0
    
    def start_move_robot_to_aruco_callback(self, msg):
            self.get_logger().info("Starting robot rotation to align with ArUco marker.")
            self.running = True
            self.counter = 0

    def listener_callback(self, msg):
        if not self.running:
            return

        alignment_error = msg.alignment_error
        distance_seperation = msg.distance_seperation

        if self.counter >= 5:
            self.get_logger().info("Robot aligned with target. Stopping rotation.")
            self.get_logger().info(f"Final Alignment Error: {alignment_error:.2f} degrees, Distance Seperation: {distance_seperation:.2f} meters")
            stop_msg = Bool()
            stop_msg.data = True
            self.publisher_stop_.publish(stop_msg)

            stop_aruco_msg = Bool()
            stop_aruco_msg.data = True
            self.publisher_stop_aruco_pose_parameters.publish(stop_aruco_msg)
            self.stop_robot()

            move_distance_msg = Float32()
            move_distance_msg.data = distance_seperation
            self.publisher_move_robot_distance.publish(move_distance_msg)

            self.destroy_node()
            rclpy.shutdown()
            return 

        if abs(alignment_error) < alignment_threshold:
            self.stop_robot()
            self.counter += 1
            return
        else:
            self.counter = 0
        angular_velocity_val = self.angular_velocity
        if alignment_error > 0:
            angular_velocity_val *= -1

        command = Twist()
        command.linear.x = 0.0
        command.linear.y = 0.0
        command.linear.z = 0.0
        command.angular.x = 0.0
        command.angular.y = 0.0
        command.angular.z = angular_velocity_val
        self.publisher_.publish(command)
    
    def stop_robot(self):
        command = Twist()
        command.linear.x = 0.0
        command.linear.y = 0.0
        command.linear.z = 0.0
        command.angular.x = 0.0
        command.angular.y = 0.0
        command.angular.z = 0.0
        self.publisher_.publish(command)


def main(args=None):
    rclpy.init(args=args)
    rotate_robot2 = RotateRobot2()
    rclpy.spin(rotate_robot2)
    rotate_robot2.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        
