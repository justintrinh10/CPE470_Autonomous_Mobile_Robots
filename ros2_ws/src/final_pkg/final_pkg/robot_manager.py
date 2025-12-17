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

        self.publisher_start_move_robot_to_point = self.create_publisher(String, 'start_move_robot_to_point', 10)
        self.subscriber_move_robot_to_point_complete = self.create_subscription(
            Bool,
            'move_robot_to_point_complete',
            self.move_robot_to_point_complete_callback,
            10
        )

        self.publisher_start_move_robot_outside_box = self.create_publisher(Bool, 'start_move_robot_outside_box', 10)
        self.subscriber_move_robot_outside_box_complete = self.create_subscription(
            Bool,
            'move_robot_outside_box_complete',
            self.move_robot_outside_box_complete_callback,
            10
        )

        self.publisher_start_move_robot_to_aruco = self.create_publisher(Int32, 'start_move_robot_to_aruco', 10)
        self.subcriber_move_robot_to_aruco_complete = self.create_subscription(
            Bool,
            'move_robot_to_aruco_complete',
            self.move_robot_to_aruco_complete_callback,
            10
        )

        self.state = "INIT"

        self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        if self.state == "INIT":
            self.get_logger().info('Robot Manager Node has been started.')
            self.get_logger().info('Starting Robot Manager...')
            self.state = "USER_INTERFACE"
            msg = Bool()
            msg.data = True
            self.publisher_start_user_interface.publish(msg)
            self.get_logger().info('User interface started. Waiting for user input...')


    def user_interface_complete_callback(self, msg):
        if self.state != "USER_INTERFACE":
            return
        self.user_point = msg.data
        self.get_logger().info(f'User interface complete. Target point: {self.user_point}')

        self.state = "MOVE_TO_POINT"
        move_point_msg = String()
        move_point_msg.data = self.user_point
        self.publisher_start_move_robot_to_point.publish(move_point_msg)
    
    def move_robot_to_point_complete_callback(self, msg):
        if self.state != "MOVE_TO_POINT":
            return
        if msg.data == True:
            self.get_logger().info('Robot moved to user-defined point successfully.')

            self.state = "MOVE_OUTSIDE_BOX"
            move_outside_box_msg = Bool()
            move_outside_box_msg.data = True
            self.publisher_start_move_robot_outside_box.publish(move_outside_box_msg)

    def move_robot_outside_box_complete_callback(self, msg):
        if self.state != "MOVE_OUTSIDE_BOX":
            return
        if msg.data == True:
            self.get_logger().info('Robot moved outside box successfully.')
            self.moving_robot_outside_box = False

            self.state = "MOVE_TO_ARUCO"
            move_to_aruco_msg = Int32()
            move_to_aruco_msg.data = aruco_id_target
            self.publisher_start_move_robot_to_aruco.publish(move_to_aruco_msg)
    
    def move_robot_to_aruco_complete_callback(self, msg):
        if self.state != "MOVE_TO_ARUCO":
            return
        if msg.data == True:
            self.get_logger().info('Robot moved to Aruco marker successfully.')
            self.state = "COMPLETE"
            self.moving_robot_to_aruco = False
            self.get_logger().info('Robot Manager sequence complete.')
    
def main(args=None):
    rclpy.init(args=args)
    robot_manager = RobotManager()
    rclpy.spin(robot_manager)
    robot_manager.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()