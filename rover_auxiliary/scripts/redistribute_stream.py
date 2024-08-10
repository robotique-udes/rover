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
        super().__init__('rtsp_server_node')
        Gst.init(None)
        self.server = None
        self.loop = None

        # Declare ROS2 parameters
        self.declare_parameter('stream_ip', '192.168.144.103')  # IP of the source stream
        self.declare_parameter('stream_port', 5884)  # Port of the source stream
        self.declare_parameter('server_port', 5884)  # Port for serving the redistributed stream
        self.declare_parameter('output_ip', '192.168.144.102')  # IP for redistribution

        # Get parameters
        self.stream_ip = self.get_parameter('stream_ip').value
        self.stream_port = self.get_parameter('stream_port').value
        self.server_port = self.get_parameter('server_port').value
        self.output_ip = self.get_parameter('output_ip').value

        self.get_logger().info(f"Initialized with stream IP: {self.stream_ip}, stream port: {self.stream_port}, "
                               f"server port: {self.server_port}, output IP: {self.output_ip}")

        # Validate IP address and start server if valid
        if self.is_valid_ip(self.stream_ip) and self.is_valid_ip(self.output_ip):
            self.start_server()
        else:
            self.get_logger().error(f"Invalid IP address detected: {self.stream_ip} or {self.output_ip}")

    def is_valid_ip(self, ip):
        try:
            socket.inet_aton(ip)
            self.get_logger().info(f"IP address {ip} is valid")
            return True
        except socket.error:
            self.get_logger().error(f"IP address {ip} is invalid")
            return False

    def start_server(self):
        self.get_logger().info("Starting RTSP server setup...")
        if self.server is None:
            self.server = GstRtspServer.RTSPServer()
            self.server.set_service(str(self.server_port))
            self.factory = GstRtspServer.RTSPMediaFactory()

            # Constructing the GStreamer pipeline
            pipeline = (
                f'rtspsrc location=rtsp://{self.stream_ip}:{self.stream_port}/test latency=0 ! '
                f'rtph264depay ! h264parse ! rtph264pay name=pay0 pt=96'
            )
            self.get_logger().info(f"GStreamer pipeline: {pipeline}")
            self.factory.set_launch(pipeline)
            self.factory.set_shared(True)

            mount_points = self.server.get_mount_points()
            mount_points.add_factory("/test", self.factory)
            self.get_logger().info("Factory added to mount points")

            attach_result = self.server.attach(None)
            if attach_result:
                self.get_logger().info("Server attached successfully")
            else:
                self.get_logger().error("Failed to attach the server")

            # Construct the full RTSP URL using the output IP
            rtsp_url = f"rtsp://{self.output_ip}:{self.server_port}/test"
            self.get_logger().info(f"RTSP server is running. Full address: {rtsp_url}")

            # Start GLib MainLoop in a separate thread
            self.loop = GLib.MainLoop()
            self.get_logger().info("Starting GLib MainLoop in a separate thread")
            self.loop_thread = threading.Thread(target=self.loop.run)
            self.loop_thread.start()
        else:
            self.get_logger().info("RTSP server is already running")

    def shutdown(self):
        self.get_logger().info("Shutting down RTSP server and GLib MainLoop")
        if self.loop:
            self.loop.quit()
        if self.server:
            self.server.detach()
        self.get_logger().info("Server detached and loop stopped")

if __name__ == '__main__':
    rclpy.init(args=None)
    # Gst.init(None)
    time.sleep(1)
    node = RTSPServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.get_logger().info("Shutting down node")
        node.destroy_node()
        rclpy.shutdown()
