import rclpy
from rclpy.node import Node
from checkpoint_interfaces.msg import ParametersToTarget
import numpy as np
from geometry_msgs.msg import Twist
import math
from std_msgs.msg import Bool

alignment_threshold = 1.5  # degrees
marker_id_target = 1  # Target ArUco marker ID

class MoveRobotToAruco(Node):
    def __init__(self):
        super().__init__('move_robot_to_aruco')
        self.subscription = self.create_subscription(
            ParametersToTarget,
            'aruco_pose_parameters',
            self.listener_callback,
            10
        )
        
        self.publisher_start_aruco_detection = self.create_publisher(Bool, 'start_aruco_detection', 10)

        

def main(args=None):
    rclpy.init(args=args)
    move_robot_to_aruco = MoveRobotToAruco()
    rclpy.spin(move_robot_to_aruco)
    move_robot_to_aruco.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        
