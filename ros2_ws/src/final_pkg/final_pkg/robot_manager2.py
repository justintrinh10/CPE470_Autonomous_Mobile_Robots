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
point1 = "0.67, 0.45"
point2 = "0.67, 1.45"

class RobotManager2(Node):
    def __init__(self):
        super().__init__('robot_manager2')

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

        self.publisher_start_move_robot_to_aruco = self.create_publisher(Int32, 'start_move_robot_to_aruco', 10)
        self.subcriber_move_robot_to_aruco_complete = self.create_subscription(
            Bool,
            'move_robot_to_aruco_complete',
            self.move_robot_to_aruco_complete_callback,
            10
        )

        self.state = "INIT"

        self.create_timer(2.0, self.timer_callback)

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
        if self.state == "MOVE_TO_POINT":
            self.get_logger().info('Robot moved to user-defined point successfully.')
            self.state = "MOVE_TO_POINT1"
            move_point_msg = String()
            move_point_msg.data = point1
            self.publisher_start_move_robot_to_point.publish(move_point_msg)
            
        elif self.state == "MOVE_TO_POINT1":
            self.get_logger().info('Robot moved to first predefined point successfully.')
            self.state = "MOVE_TO_POINT2"
            move_point_msg = String()
            move_point_msg.data = point2
            self.publisher_start_move_robot_to_point.publish(move_point_msg)
        elif self.state == "MOVE_TO_POINT2":
            self.get_logger().info('Robot moved to second predefined point successfully.')
            self.state = "MOVE_TO_ARUCO"

            aruco_msg = Int32()
            aruco_msg.data = aruco_id_target
            self.publisher_start_move_robot_to_aruco.publish(aruco_msg)

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
    robot_manager2 = RobotManager2()
    rclpy.spin(robot_manager2)
    robot_manager2.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()