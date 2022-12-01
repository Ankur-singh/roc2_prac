import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber')
        self.subscriber_ = self.create_subscription(String, 'bot_updates', self.callback, 10)
        self.get_logger().info('Subscriber Started . . . ')

    def callback(self, msg):
        self.get_logger().info(msg.data)
        

def main(args=None):
    rclpy.init(args=args)
    node = SubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()