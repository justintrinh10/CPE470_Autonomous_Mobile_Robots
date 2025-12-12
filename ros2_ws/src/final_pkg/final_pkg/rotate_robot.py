import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist
import math
from std_msgs.msg import Float32
from std_msgs.msg import Bool

class RotateRobot(Node):
    def __init__(self):
        super().__init__('rotate_robot')
        self.subscriber_ = self.create_subscription(
            Float32,
            'rotate_robot_angle',
            self.listener_callback,
            10
        )

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.publisher_stop_ = self.create_publisher(Bool, 'rotate_robot_angle_complete', 10)
        self.angular_velocity = 0.005  # radians per second
        self.current_angle = 0.0  # degrees
        self.desired_angle = 0.0  # degrees
        self.rotate_complete = False

    def listener_callback(self, msg):
        if self.rotate_complete:
            return
        
        

        if self.counter >= 5:
            self.get_logger().info("Robot aligned with target. Stopping rotation.")
            self.get_logger().info(f"Final Alignment Error: {alignment_error:.2f} degrees, Distance Seperation: {distance_seperation:.2f} meters")
            stop_msg = Bool()
            stop_msg.data = True
            self.publisher_stop_.publish(stop_msg)
            self.stop_robot()
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
    rotate_robot = RotateRobot()
    rclpy.spin(rotate_robot)
    rotate_robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        
