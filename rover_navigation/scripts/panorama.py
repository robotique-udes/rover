#!/usr/bin/env python3

import rclpy
from rclpy.node import Node 
import cv2
from std_msgs.msg import Bool
import os
from datetime import datetime, timedelta
import threading
from ament_index_python.packages import get_package_share_directory

class PanoramaNode(Node):
    def __init__(self):
        super().__init__('panorama_node')
        self.subscription = self.create_subscription(
            Bool,
            '/rover/auxiliary/panorama/start',
            self.button_pressed_callback,
            10)
        self.ip = "rtsp://127.0.0.1:8554/stream"
        self.is_capturing = False
        self.frame_count = 0
        self.save_directory = self.get_resources_directory('rover_navigation')
        os.makedirs(self.save_directory, exist_ok=True)
        self.capture_thread = None
        self.timeout_duration = timedelta(seconds=30) 

    def get_resources_directory(self, package_name):
        package_share_directory = get_package_share_directory(package_name)
        resource_directory = package_share_directory + "/resource/"
        return resource_directory
    
    def button_pressed_callback(self, msg):
        if msg.data and not self.is_capturing:
            self.get_logger().info('Starting panorama capture...')
            self.is_capturing = True
            self.capture_thread = threading.Thread(target=self.capture_frames)
            self.capture_thread.start()
        elif not msg.data and self.is_capturing:
            self.get_logger().info('Stopping panorama capture...')
            self.is_capturing = False
            if self.capture_thread:
                self.capture_thread.join()

    def capture_frames(self):
        try:
            cap = cv2.VideoCapture(self.ip)
            if not cap.isOpened():
                self.get_logger().error('Failed to open video stream')
                self.is_capturing = False
                return

            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            session_directory = os.path.join(self.save_directory, timestamp)
            os.makedirs(session_directory, exist_ok=True)

            self.frame_count = 0
            start_time = datetime.now()
            while self.is_capturing:
                ret, frame = cap.read()
                if ret:
                    self.frame_count += 1
                    frame_filename = os.path.join(session_directory, f"frame_{self.frame_count:04d}.jpg")
                    cv2.imwrite(frame_filename, frame)
                    self.get_logger().info(f'Saved frame {self.frame_count} to {frame_filename}')
                else:
                    self.get_logger().warn("Failed to capture frame")

                if datetime.now() - start_time > self.timeout_duration:
                    self.get_logger().info('Capture session timed out')
                    self.is_capturing = False
                    break

            cap.release()
            self.get_logger().info(f"Capture session ended. Total frames captured: {self.frame_count}")
        except Exception as e:
            self.get_logger().error(f"An error occurred during frame capture: {str(e)}")
            self.is_capturing = False
            if cap.isOpened():
                cap.release()
            self.get_logger().info(f"Capture session ended due to error. Total frames captured: {self.frame_count}")

def main():
    rclpy.init()
    panorama_node = PanoramaNode()
    rclpy.spin(panorama_node)
    panorama_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
