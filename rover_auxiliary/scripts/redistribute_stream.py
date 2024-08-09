#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstRtspServer', '1.0')
from gi.repository import Gst, GstRtspServer, GLib
import socket
import time
import threading

class RTSPServerNode(Node):
    def __init__(self):
        print("test 3")
        super().__init__('rtsp_server_node')
        Gst.init(None)
        self.server = None
        self.loop = None
        
        # Declare ROS2 parameters
        self.declare_parameter('stream_ip', '127.0.0.2')
        self.declare_parameter('stream_port', 8554)
        self.declare_parameter('server_port', 8500)
        
        # Get parameters
        self.stream_ip = self.get_parameter('stream_ip').value
        self.stream_port = self.get_parameter('stream_port').value
        self.server_port = self.get_parameter('server_port').value
        
        # Validate IP address and start server if valid
        if self.is_valid_ip(self.stream_ip):
            self.start_server()
        else:
            self.get_logger().error(f"Invalid IP address: {self.stream_ip}")

    def is_valid_ip(self, ip):
        try:
            socket.inet_aton(ip)
            return True
        except socket.error:
            return False

    def start_server(self):
        if self.server is None:
            self.server = GstRtspServer.RTSPServer()
            self.server.set_service(str(self.server_port))
            
            self.factory = GstRtspServer.RTSPMediaFactory()
            pipeline = f'( rtspsrc location=rtsp://{self.stream_ip}:{self.stream_port}/test latency=0 ! rtph264depay ! h264parse ! rtph264pay name=pay0 pt=96 )'
            self.factory.set_launch(pipeline)
            self.factory.set_shared(True)
            
            self.server.get_mount_points().add_factory("/test", self.factory)
            self.server.attach(None)
            
            self.get_logger().info(f"RTSP server is running at rtsp://127.0.0.1:{self.server_port}/test")
            
            # Start GLib MainLoop in a separate thread
            self.loop = GLib.MainLoop()
            self.loop_thread = threading.Thread(target=self.loop.run)
            self.loop_thread.start()
        else:
            self.get_logger().info("RTSP server is already running")

if __name__ == '__main__':
    # Gst.init(None)
    print("test 1")
    rclpy.init(args=None)
    time.sleep(1)
    print("test 2")
    node = RTSPServerNode()
    print("test 4")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean shutdown
        if node.loop:
            node.loop.quit()
        if node.server:
            node.server.detach()
        node.destroy_node()
        rclpy.shutdown()