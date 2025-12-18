import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool, Float32
from rclpy.qos import QoSProfile, ReliabilityPolicy
import math


class Navigator(Node):
    def __init__(self):
        super().__init__('navigator')

        # ---------- Goal ----------
        self.goal_x = 0.0
        self.goal_y = 0.0

        # ---------- State ----------
        self.position = None
        self.yaw = None
        self.goal_received = False
        self.arrived = False
        self.running_distance_move = False
        self.rotated = False
        self.moved_complete = False

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.done_pub = self.create_publisher(Bool, 'move_robot_to_point_complete', 10)
        self.move_dist_pub = self.create_publisher(Float32, 'move_robot_distance', 10)

        self.position_sub = self.create_subscription(
            String, 'robot_position', self.position_cb, 10
        )

        self.start_sub = self.create_subscription(
            String, 'start_move_robot_to_point', self.start_move_robot_callback, 10
        )

        self.move_dist_done_sub = self.create_subscription(
            Bool, 'move_robot_distance_complete', self.move_distance_done_callback, 10
        )

        odom_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_cb, odom_qos
        )

        # Timer for navigation loop
        self.timer = self.create_timer(0.1, self.navigate)

    def position_cb(self, msg):
        try:
            x, y = msg.data.split(',')
            self.position = (float(x), float(y))
        except:
            self.get_logger().warn("Navigator: Bad robot_position format")

    def start_move_robot_callback(self, msg):
        try:
            x, y = msg.data.split(',')
            self.goal_x = float(x)
            self.goal_y = float(y)
            self.goal_received = True
            self.arrived = False
            self.rotated = False
            self.moved_complete = False
            self.running_distance_move = False
            self.get_logger().info(f"Navigator: New goal ({self.goal_x}, {self.goal_y})")
        except:
            self.get_logger().warn("Navigator: Bad goal format")

    def move_distance_done_callback(self, msg):
        if self.arrived:
            return
        if msg.data:
            self.running_distance_move = False
            self.moved_complete = True
            self.arrived = True
            self.get_logger().info("Navigator: Distance movement complete")
            done_msg = Bool()
            done_msg.data = True
            self.done_pub.publish(done_msg)
            Twist_cmd = Twist()
            self.cmd_pub.publish(Twist_cmd)

    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def navigate(self):
        if not self.goal_received or self.position is None or self.yaw is None:
            return

        if self.arrived:
            return

        x, y = self.position
        dx = self.goal_x - x
        dy = self.goal_y - y
        dist = math.hypot(dx, dy)

        angle_to_goal = math.atan2(dy, dx)
        angle_error = math.atan2(
            math.sin(angle_to_goal - self.yaw),
            math.cos(angle_to_goal - self.yaw)
        )

        if abs(angle_error) > 0.3 and not self.rotated:
            cmd = Twist()
            cmd.angular.z = 2.0 * angle_error
            self.cmd_pub.publish(cmd)
            return
        elif not self.rotated:
            self.rotated = True
            cmd = Twist()
            self.cmd_pub.publish(cmd)
        else:
            self.rotated = True

            if not self.running_distance_move:
                msg = Float32()
                msg.data = dist

                self.move_dist_pub.publish(msg)
                self.running_distance_move = True

                self.get_logger().info(f"Navigator: Moving {dist:.2f} m toward goal")


def main():
    rclpy.init()
    node = Navigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
