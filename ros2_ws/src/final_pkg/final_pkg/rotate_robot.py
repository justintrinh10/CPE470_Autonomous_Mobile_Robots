import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist
import math
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from irobot_create_msgs.msg import WheelTicks
from rclpy.qos import QoSProfile, ReliabilityPolicy

class RotateRobot(Node):
    def __init__(self):
        super().__init__('rotate_robot')
        self.subscriber_ = self.create_subscription(
            Float32,
            'rotate_robot_angle',
            self.listener_callback_rotate_robot_angle,
            10
        )

        wheel_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.subscription_wheel_enconder = self.create_subscription(
            WheelTicks,
            '/wheel_ticks',
            self.listener_callback_wheel_encoder,
            wheel_qos
        )

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.publisher_stop_ = self.create_publisher(Bool, 'rotate_robot_angle_complete', 10)
        self.angular_velocity = 0.005  # radians per second
        self.current_angle = 0.0  # degrees
        self.desired_angle = 0.0  # degrees
        self.rotate_complete = False
        self.wheel_seperation = 0.235  # meters
        self.ticks_per_revolution = 508.8
        self.wheel_radius = 0.036 # meters
        self.prev_ticks_left = 0
        self.prev_ticks_right = 0

    def listener_callback_rotate_robot_angle(self, msg):
        self.desired_angle = msg.data
        self.current_angle = 0.0
        self.rotate_complete = False
        
    def listener_callback_wheel_encoder(self, msg):
        if self.rotate_complete:
            return

        left_ticks = msg.ticks_left
        right_ticks = msg.ticks_right
        if self.prev_ticks_left == 0 and self.prev_ticks_right == 0:
            self.prev_ticks_left = left_ticks
            self.prev_ticks_right = right_ticks
            return
        ticks_difference_left = left_ticks - self.prev_ticks_left
        ticks_difference_right = right_ticks - self.prev_ticks_right
        self.prev_ticks_left = left_ticks
        self.prev_ticks_right = right_ticks
        dist_moved_left = (ticks_difference_left / self.ticks_per_revolution) * (2 * math.pi * self.wheel_radius)
        dist_moved_right = (ticks_difference_right / self.ticks_per_revolution) * (2 * math.pi * self.wheel_radius)
        angle_turned = (dist_moved_right - dist_moved_left) / self.wheel_seperation
        angle_turned_degrees = math.degrees(angle_turned)
        self.current_angle += angle_turned_degrees

        if abs(self.current_angle) >= self.desired_angle:
            self.get_logger().info("Robot rotated desired angle. Stopping rotation.")
            self.get_logger().info(f"Total Angle Rotated: {self.current_angle:.2f} degrees")
            self.stop_robot()
            self.rotate_complete = True
            complete_msg = Bool()
            complete_msg.data = True
            self.publisher_stop_.publish(complete_msg)
            return
        
        angular_velocity_val = self.angular_velocity
        if self.desired_angle > 0:
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
        
