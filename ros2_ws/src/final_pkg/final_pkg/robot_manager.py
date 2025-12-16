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

aruco_id_target = 1

class RobotManager(Node):
    def __init__(self):
        super().__init__('robot_manager')

        self.publisher_start_user_interface = self.create_publisher(Bool, 'start_user_interface', 10)
        self.subscriber_user_interface_complete = self.create_subscription(
            String,
            'user_interface_complete',
            self.user_interface_complete_callback,
            10
        )
        self.user_point =  ""
        self.user_interface_running = False

        self.publisher_start_move_robot_to_point = self.create_publisher(String, 'start_move_robot_to_point', 10)
        self.subscriber_move_robot_to_point_complete = self.create_subscription(
            Bool,
            'move_robot_to_point_complete',
            self.move_robot_to_point_complete_callback,
            10
        )
        self.moving_robot_to_point = False

        self.publisher_start_move_robot_outside_box = self.create_publisher(Bool, 'start_move_robot_outside_box', 10)
        self.subscriber_move_robot_outside_box_complete = self.create_subscription(
            Bool,
            'move_robot_outside_box_complete',
            self.move_robot_outside_box_complete_callback,
            10
        )
        self.moving_robot_outside_box = False

        self.publisher_start_move_robot_to_aruco = self.create_publisher(Int32, 'start_move_robot_to_aruco', 10)
        self.subcriber_move_robot_to_aruco_complete = self.create_subscription(
            Bool,
            'move_robot_to_aruco_complete',
            self.move_robot_to_aruco_complete_callback,
            10
        )
        self.moving_robot_to_aruco = False

        self.get_logger().info('Robot Manager Node has been started.')
        self.start_robot_manager()

    def start_robot_manager(self):
        self.get_logger().info('Starting Robot Manager...')
        msg = Bool()
        msg.data = True
        self.publisher_start_user_interface.publish(msg)
        self.user_interface_running = True
        self.get_logger().info('User interface started. Waiting for user input...')

        while self.user_interface_running == True:
            rclpy.spin_once(self)
        self.get_logger().info(f'User input received: {self.user_point}')
        
        move_point_msg = String()
        move_point_msg.data = self.user_point
        self.publisher_start_move_robot_to_point.publish(move_point_msg)
        self.moving_robot_to_point = True

        while self.moving_robot_to_point == True:
            rclpy.spin_once(self)
        
        move_outside_box_msg = Bool()
        move_outside_box_msg.data = True
        self.publisher_start_move_robot_outside_box.publish(move_outside_box_msg)
        self.moving_robot_outside_box = True

        while self.moving_robot_outside_box == True:
            rclpy.spin_once(self)

        move_to_aruco_msg = Int32()
        move_to_aruco_msg.data = aruco_id_target
        self.publisher_start_move_robot_to_aruco.publish(move_to_aruco_msg)
        self.moving_robot_to_aruco = True

        while self.moving_robot_to_aruco == True:
            rclpy.spin_once(self)
        
        self.get_logger().info('Robot Manager tasks completed.')

    def user_interface_complete_callback(self, msg):
        self.user_point = msg.data
        self.user_interface_running = False
        self.get_logger().info(f'User interface complete. Target point: {self.user_point}')
    
    def move_robot_to_point_complete_callback(self, msg):
        if msg.data == True:
            self.get_logger().info('Robot moved to user-defined point successfully.')
            self.moving_robot_to_point = False

    def move_robot_outside_box_complete_callback(self, msg):
        if msg.data == True:
            self.get_logger().info('Robot moved outside box successfully.')
            self.moving_robot_outside_box = False
    
    def move_robot_to_aruco_complete_callback(self, msg):
        if msg.data == True:
            self.get_logger().info('Robot moved to Aruco marker successfully.')
            self.moving_robot_to_aruco = False
    
def main(args=None):
    rclpy.init(args=args)
    robot_manager = RobotManager()
    rclpy.spin(robot_manager)
    robot_manager.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()