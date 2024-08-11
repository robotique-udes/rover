#!/usr/bin/env python3
import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstRtspServer', '1.0')
from gi.repository import Gst, GstRtspServer, GLib
import rclpy
from rclpy.node import Node
import socket
import time
import threading

URL_NAME:str = "cam"

class RTSPServerNode(Node):
    def __init__(self):
        super().__init__('rtsp_server_node')
        Gst.init(None)
        self.server1 = None
        self.server2 = None
        self.loop = None
        self.declare_parameter('flip_video', True)
        self.flip_video = self.get_parameter('flip_video').value

        self.declare_parameter('streamIP', '192.168.144.62')  
        self.declare_parameter('streamPort', 69)  
        self.declare_parameter('serverPort1', 8554)  
        self.declare_parameter('serverPort2', 8555)  

        self.streamIP = self.get_parameter('streamIP').value
        self.streamPort = self.get_parameter('streamPort').value
        self.streamPort1 = self.get_parameter('serverPort1').value
        self.streamPort2 = self.get_parameter('serverPort2').value

        self.get_logger().debug(f"Initialized with stream IP: {self.streamIP}, stream port: {self.streamPort}, "
                               f"server ports: {self.streamPort1} and {self.streamPort2}")

        if self.IPisValid(self.streamIP):
            self.startServers()
        else:
            self.get_logger().error(f"Invalid IP address detected: {self.streamIP}")

    def IPisValid(self, ip):
        try:
            socket.inet_aton(ip)
            self.get_logger().debug(f"IP address {ip} is valid")
            return True
        except socket.error:
            self.get_logger().error(f"IP address {ip} is invalid")
            return False

    def startServers(self):
        self.get_logger().debug("Starting RTSP server setup...")

        if self.server1 is None:
            self.server1 = GstRtspServer.RTSPServer()
            self.server1.set_service(str(self.streamPort1))
            factory1 = GstRtspServer.RTSPMediaFactory()
            
            flip_element = "videoflip method=horizontal-flip ! " if self.flip_video else ""

            pipeline = (
                f'rtspsrc location=rtsp://admin:admin@{self.streamIP}:{self.streamPort} latency=0 ! '
                f'rtph264depay ! h264parse ! decodebin ! {flip_element}'
                f'x264enc tune=zerolatency ! rtph264pay name=pay0 pt=96'
            )
            
            self.get_logger().debug(f"GStreamer pipeline for first stream: {pipeline}")
            factory1.set_launch(pipeline)
            factory1.set_shared(True)

            mountPoints1 = self.server1.get_mount_points()
            mountPoints1.add_factory("/cam", factory1)
            self.get_logger().debug("First factory added to mount points")

            attach_result1 = self.server1.attach(None)
            if attach_result1:
                self.get_logger().debug("First server attached successfully")
            else:
                self.get_logger().error("Failed to attach the first server")

            rtsp_url1 = f"rtsp://THIS_PC_IP:{self.streamPort1}/cam"
            self.get_logger().info(f"First RTSP server is running. Full address: {rtsp_url1}")
        
        if self.server2 is None:
            self.server2 = GstRtspServer.RTSPServer()
            self.server2.set_service(str(self.streamPort2))
            factory2 = GstRtspServer.RTSPMediaFactory()

            self.get_logger().debug(f"GStreamer pipeline for second stream: {pipeline}")
            factory2.set_launch(pipeline)
            factory2.set_shared(True)

            mountPoints2 = self.server2.get_mount_points()
            mountPoints2.add_factory("/cam", factory2)
            self.get_logger().debug("Second factory added to mount points")

            attach_result2 = self.server2.attach(None)
            if attach_result2:
                self.get_logger().debug("Second server attached successfully")
            else:
                self.get_logger().error("Failed to attach the second server")

            rtsp_url2 = f"rtsp://THIS_PC_IP:{self.streamPort2}/cam"
            self.get_logger().info(f"Second RTSP server is running. Full address: {rtsp_url2}")

        if self.loop is None:
            self.loop = GLib.MainLoop()
            self.get_logger().info("Starting GLib MainLoop in a separate thread")
            self.loop_thread = threading.Thread(target=self.loop.run, daemon=True)
            self.loop_thread.start()

if __name__ == '__main__':
    rclpy.init(args=None)
    node = RTSPServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
