import rclpy
from rclpy.node import Node
from checkpoint_interfaces.msg import ParametersToTarget
import numpy as np
from geometry_msgs.msg import Twist
import math
from std_msgs.msg import Bool

alignment_threshold = 3.0  # degrees

class RotateRobot(Node):
    def __init__(self):
        super().__init__('rotate_robot')
        self.subscription = self.create_subscription(
            ParametersToTarget,
            'aruco_pose_parameters',
            self.listener_callback,
            10
        )
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.publisher_stop_ = self.create_publisher(Bool, 'rotation_complete', 10)
        self.angular_velocity = 0.005  # radians per second

    def listener_callback(self, msg):
        alignment_error = msg.alignment_error
        distance_seperation = msg.distance_seperation

        if abs(alignment_error) < alignment_threshold:
            self.get_logger().info("Robot aligned with target. Stopping rotation.")
            self.get_logger().info(f"Final Alignment Error: {alignment_error:.2f} degrees, Distance Seperation: {distance_seperation:.2f} meters")
            stop_msg = Bool()
            stop_msg.data = True
            self.publisher_stop_.publish(stop_msg)
            rclpy.shutdown()
            self.destroy_node()
            return

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
            
    
    def destroy_node(self):
        command = Twist()
        command.linear.x = 0.0
        command.linear.y = 0.0
        command.linear.z = 0.0
        command.angular.x = 0.0
        command.angular.y = 0.0
        command.angular.z = 0.0
        self.publisher_.publish(command)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    rotate_robot = RotateRobot()
    rclpy.spin(rotate_robot)

if __name__ == '__main__':
    main()
        
