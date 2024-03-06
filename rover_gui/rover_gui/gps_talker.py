import rclpy
from rclpy.node import Node

from rover_msgs.msg import Gps

class GpsTalker(Node):
    def __init__(self):
        super().__init__('gps_talker')
        self.publisher_ = self.create_publisher(Gps, 'gps_talker', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Gps()
        msg.latitude = float(self.i)
        msg.longitude = self.i + 1.222
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "lat: %f , lon: %f' % (msg.latitude, msg.longitude))
        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    talker = GpsTalker()
    rclpy.spin(talker)

    rclpy.shutdown()