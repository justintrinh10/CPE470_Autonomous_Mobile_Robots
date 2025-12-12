import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist
import math
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from irobot_create_msgs.msg import WheelTicks
from rclpy.qos import QoSProfile, ReliabilityPolicy

class MoveRobot(Node):
    def __init__(self):
        super().__init__('move_robot')

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

        self.subscription_move = self.create_subscription(
            Float32,
            'move_robot_distance',
            self.listener_callback_move_robot_distance,
            10
        )

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.publisher_move_robot_distance_complete = self.create_publisher(Bool, 'move_robot_distance_complete', 10)
        self.linear_velocity = 0.20  # meters per second
        self.ticks_per_revolution = 508.8
        self.wheel_radius = 0.036 # meters
        self.distance_traveled = 0.0 #meters
        self.desired_distance = 0.0 #meters
        self.move_complete = False
        self.prev_ticks = 0

    def listener_callback_move_robot_distance(self, msg):
        self.desired_distance = msg.data
        self.distance_traveled = 0.0
        self.move_complete = False
    
    def listener_callback_wheel_encoder(self, msg):
        if self.move_complete:
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

        if self.distance_traveled >= self.desired_distance:
            self.get_logger().info("Robot traveled desired distance. Stopping movement.")
            self.get_logger().info(f"Total Distance Traveled: {self.distance_traveled:.2f} meters")
            self.stop_robot()
            self.move_complete = True
            complete_msg = Bool()
            complete_msg.data = True
            self.publisher_move_robot_distance_complete.publish(complete_msg)
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

