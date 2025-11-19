import rclpy
from rclpy.node import Node
from checkpoint_interfaces.msg import ParametersToTarget
import numpy as np
from geometry_msgs.msg import Twist
import math
from std_msgs.msg import Bool
from irobot_create_msgs.msg import WheelTicks

desired_distance = 0.30  # meters

class MoveRobot(Node):
    def __init__(self):
        super().__init__('move_robot')
        self.subscription_parameters = self.create_subscription(
            ParametersToTarget,
            'aruco_pose_parameters',
            self.listener_callback_parameters,
            10
        )
        self.subscription_rotation_complete = self.create_subscription(
            Bool,
            'rotation_complete',
            self.listener_callback_rotation_complete,
            10
        )
        self.subscription_wheel_enconder = self.create_subscription(
            WheelTicks,
            '/wheel_ticks',
            self.listener_callback_wheel_encoder,
            10
        )

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.linear_velocity = 0.20  # meters per second
        self.distance_seperation = 0.0 # meters
        self.ticks_per_revolution = 508.8
        self.wheel_radius = 0.036 # meters
        self.distance_traveled = 0.0 #meters
        self.rotation_complete = False
        self.prev_ticks = 0

    def listener_callback_parameters(self, msg):
        self.distance_seperation = msg.distance_seperation
    
    def listener_callback_rotation_complete(self, msg):
        if msg.data:
            self.rotation_complete = True
    
    def listener_callback_wheel_encoder(self, msg):
        if not self.rotation_complete:
            return
        if self.distance_traveled >= desired_distance:
            self.stop_robot()
            return

        left_ticks = msg.ticks_left
        right_ticks = msg.ticks_right
        average_ticks = (left_ticks + right_ticks) / 2.0
        if self.prev_ticks == 0:
            self.prev_ticks = average_ticks
            return
        ticks_difference = average_ticks - self.prev_ticks
        self.prev_ticks = average_ticks
        dist_moved = (ticks_difference / self.ticks_per_revolution) * (2 * math.pi * self.wheel_radius)
        self.distance_traveled += dist_moved

        if self.distance_traveled >= desired_distance:
            self.get_logger().info("Robot traveled desired distance. Stopping movement.")
            self.get_logger().info(f"Distance Seperation: {self.distance_seperation:.2f} meters")
            self.get_logger().info(f"Total Distance Traveled: {self.distance_traveled:.2f} meters")
            self.stop_robot()
            self.destroy_node()
            rclpy.shutdown()
            return

        command = Twist()
        command.linear.x = self.linear_velocity
        command.linear.y = 0.0
        command.linear.z = 0.0
        command.angular.x = 0.0
        command.angular.y = 0.0
        command.angular.z = 0.0
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
    move_robot = MoveRobot()
    rclpy.spin(move_robot)
    move_robot.stop_robot()
    move_robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

