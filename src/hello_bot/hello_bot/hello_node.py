import rclpy
from rclpy.node import Node

class HelloNode(Node):
	def __init__(self):
		super().__init__('hello_node')
		self.get_logger().info('Hello from ROS2!')

def main():
	rclpy.init(args=None)
	node = HelloNode()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == "__main__":
	main()