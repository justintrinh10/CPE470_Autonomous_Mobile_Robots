import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class ProductSubscriber(Node):
    def __init__(self):
        super().__init__('product_subscriber')
        self.get_logger().info('Creating Product Subscriber')
        self.current_product = 1
        self.subscription = self.create_subscription(Int32, "user_input", self.listener_callback, 10)
        self.subscription

    def listener_callback(self, msg):
        self.current_product *= msg.data
        self.get_logger().info('Current Total Product = %i' % self.current_product)

def main(args=None):
    rclpy.init(args=args)
    product_subscriber = ProductSubscriber()
    rclpy.spin(product_subscriber)
    product_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()



