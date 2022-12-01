import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher')
        self.publisher_ = self.create_publisher(String, 'bot_updates', 10)
        self.timer_ = self.create_timer(1.0, self.publish_updates)
        self.get_logger().info('Publisher Started . . . ')

    def publish_updates(self):
        msg = String()
        msg.data = "Hi, here are the new updates"
        self.get_logger().info(msg.data)
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()