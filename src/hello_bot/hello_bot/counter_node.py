import rclpy
from rclpy.node import Node

class CounterNode(Node):
	def __init__(self):
		super().__init__('counter_node')
		self.counter = 0
		self.create_timer(1.0, self.callback)

	def callback(self):
		self.get_logger().info(f"Hello {self.counter}")
		self.counter += 1

def main():
	rclpy.init(args=None)
	node = CounterNode()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == "__main__":
	main()