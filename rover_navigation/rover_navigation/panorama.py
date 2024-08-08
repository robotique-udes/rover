#!/usr/bin/env python3

import rclpy
from rclpy.node import Node 

class PanoramaNode(Node):
    def __init__(self):
        super().__init__('panorama_node')
        self.subscription = self.create_subscription(
            bool,
            '/rover/auxiliary/panorama/start',
            self.button_pressed_callback,
            10)

    def button_pressed_callback(self, msg):
        self.get_logger().info('Button press detected, starting panorama...')
        
        if msg.data:
            return None

def main():
    rclpy.init()
    panorama_node = PanoramaNode()
    rclpy.spin(panorama_node)
    panorama_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()