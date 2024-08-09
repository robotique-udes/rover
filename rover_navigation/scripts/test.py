import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import time

class PanoramaTrigger(Node):
    def __init__(self):
        super().__init__('panorama_trigger')
        self.publisher = self.create_publisher(Bool, '/rover/auxiliary/panorama/start', 10)

    def publish_message(self, data):
        msg = Bool()
        msg.data = data
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {data}')

def main(args=None):
    rclpy.init(args=args)
    node = PanoramaTrigger()

    # Publish True to start capturing
    node.publish_message(True)

    # Wait for 10 seconds
    time.sleep(10)

    # Publish False to stop capturing
    node.publish_message(False)

    # Clean up
    rclpy.shutdown()

if __name__ == '__main__':
    main()
