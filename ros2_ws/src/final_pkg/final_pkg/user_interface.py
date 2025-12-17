import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Bool
import threading

class UserInterface(Node):
    def __init__(self):
        super().__init__("user_interface")
        self.publisher_ = self.create_publisher(String, "user_interface_complete", 10)
        self.create_subscription(Bool, "start_user_interface", self.listener_callback, 10)
        self.get_logger().info("User Interface Node has been started.")

    def listener_callback(self, msg):
        self.get_logger().info("Received start signal for user interface.")
        threading.Thread(target=self.get_user_input).start()

    def get_user_input(self):
        self.get_logger().info("Enter desired point (meters)")
        x = float(input("X: "))
        y = float(input("Y: "))
        point_msg = String()
        point_msg.data = f"{x},{y}"
        self.publisher_.publish(point_msg)
        self.get_logger().info(f"Published Point: ({x}, {y})")

def main(args=None):
    rclpy.init(args=args)
    node = UserInterface()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
