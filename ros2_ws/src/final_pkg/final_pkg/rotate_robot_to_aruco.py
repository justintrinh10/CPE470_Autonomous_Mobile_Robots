import rclpy
from rclpy.node import Node
from checkpoint_interfaces.msg import ParametersToTarget
import numpy as np
from geometry_msgs.msg import Twist
import math
from std_msgs.msg import Bool

alignment_threshold = 1.5  # degrees

class RotateRobot(Node):
    def __init__(self):
        super().__init__('rotate_robot')
        self.subscription = self.create_subscription(
            ParametersToTarget,
            'aruco_pose_parameters',
            self.listener_callback,
            10
        )
        self.subscription_start = self.create_subscription(
            Bool,
            'start_rotate_robot',
            self.start_callback,
            10
        )
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.publisher_stop_ = self.create_publisher(Bool, 'rotation_complete', 10)
        self.angular_velocity = 0.005  # radians per second
        self.counter = 0
        self.Running = False
    
    def start_callback(self, msg):
        self.Running = msg.data
        self.get_logger().info(f"Rotate Robot Started: {self.Running}")

    def listener_callback(self, msg):
        if not self.Running:
            return
        
        alignment_error = msg.alignment_error
        distance_seperation = msg.distance_seperation

        if self.counter >= 5:
            self.get_logger().info("Robot aligned with target. Stopping rotation.")
            self.get_logger().info(f"Final Alignment Error: {alignment_error:.2f} degrees, Distance Seperation: {distance_seperation:.2f} meters")
            self.Running = False
            stop_msg = Bool()
            stop_msg.data = True
            self.publisher_stop_.publish(stop_msg)
            self.stop_robot()
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
    rotate_robot = RotateRobot()
    rclpy.spin(rotate_robot)
    rotate_robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        
