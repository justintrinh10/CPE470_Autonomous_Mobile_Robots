import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
 
class InputPublisher(Node):
    def __init__(self):
        super().__init__('input_publisher')
        self.get_logger().info('Creating Input Publisher')
        self.publisher_ = self.create_publisher(Int32, "user_input", 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        user_num_str = input("Enter a number:")
        user_num_int = int(user_num_str)
        msg = Int32()
        msg.data = user_num_int
        self.publisher_.publish(msg)
        self.get_logger().info('Published: %i' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    input_publisher = InputPublisher()
    rclpy.spin(input_publisher)
    input_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()