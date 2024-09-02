#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rover_msgs.srv import ScreenshotControl
from rover_msgs.msg import GpsPosition
import cv2
import os
from datetime import datetime
from ament_index_python.packages import get_package_share_directory

class ScreenshotService(Node):
    def __init__(self):
        super().__init__('screenshot_node')

        self.save_directory = self.get_resources_directory('rover_auxiliary')
        os.makedirs(self.save_directory, exist_ok=True)

        self.gps_data = None
        self.sub = self.create_subscription(GpsPosition, '/rover/gps/position', self.gps_callback, 10)
        
        self.name = None
        self.ip = None
        self.cap = None
        self.srv = self.create_service(ScreenshotControl, 'control_screenshot', self.handle_service)

    def get_resources_directory(self, package_name):
        package_share_directory = get_package_share_directory(package_name)
        resource_directory = package_share_directory + "/../../../../../resource/"
        return resource_directory

    def handle_service(self, request, response):
        self.ip = request.ip_address
        self.name = request.name
        if request.start:
            self.get_logger().info('Taking picture')
            response.success = self.take_photo()
            response.status_message = 'Photo taken.' if response.success else 'Failed to take photo.'
        return response

    def release_camera(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None

    def take_photo(self):
        try:
            self.cap = cv2.VideoCapture(self.ip)
            if not self.cap.isOpened():
                self.get_logger().error('Failed to open video stream')
                return False

            ret, frame = self.cap.read()
            if ret:
                timestamp = datetime.now().strftime("%Y%m%d_%Hh%Mmin%Ss")
                if self.name is None: self.name = ""
                if self.gps_data is None: self.gps_data = "GPS_OFFLINE"
                photo_filename = os.path.join(self.save_directory, f"{timestamp} {self.gps_data} {self.name}.jpg")
                cv2.imwrite(photo_filename, frame)
                self.get_logger().info(f'Took photo and saved to {photo_filename}')
                return True
            else:
                self.get_logger().warn("Failed to capture photo")
                return False
        except cv2.error as e:
            self.get_logger().error(f"An error occurred while taking a photo: {str(e)}")
            return False
        finally:
            self.release_camera()

    def gps_callback(self, msg):
        self.gps_data = msg

def main(args=None):
    try:
        rclpy.init(args=args)
        
        node = ScreenshotService()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
