#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.task import Future
from rover_msgs.srv import PanoControl
from rover_msgs.msg import GpsPosition
import cv2
import os
from datetime import datetime, timedelta
import threading
from ament_index_python.packages import get_package_share_directory

class PanoramaService(Node):
    def __init__(self):
        super().__init__('panorama_node')
        self.srv = self.create_service(PanoControl, 'control_panorama', self.handle_service)
        self.capture_event = threading.Event()
        self.frame_count = 0
        self.frame_count_lock = threading.Lock()
        self.save_directory = self.get_resources_directory('rover_navigation')
        os.makedirs(self.save_directory, exist_ok=True)
        self.capture_thread = None
        self.timeout_duration = timedelta(seconds=30)

    def get_resources_directory(self, package_name):
        package_share_directory = get_package_share_directory(package_name)
        resource_directory = package_share_directory + "/resource/"
        return resource_directory

    def handle_service(self, request, response):
        self.ip = request.ip_address
        if request.photo:
            self.get_logger().info('Taking picture')
            response.success = self.take_photo()
            response.status_message = 'Photo taken.' if response.success else 'Failed to take photo.'
        elif request.start and not self.capture_event.is_set():
            self.get_logger().info('Starting panorama capture...')
            self.capture_event.set()
            self.capture_thread = threading.Thread(target=self.capture_frames)
            self.capture_thread.start()
            response.success = True
            response.status_message = 'Panorama capture started.'
            self.get_logger().info('Panorama capture started')
        elif request.stop and self.capture_event.is_set():
            self.get_logger().info('Stopping panorama capture...')
            self.capture_event.clear()
            if self.capture_thread:
                self.capture_thread.join(timeout=5)  
                if self.capture_thread.is_alive():
                    self.get_logger().warn('Capture thread did not terminate gracefully')
            response.success = True
            response.status_message = 'Panorama capture stopped.'
            self.get_logger().info('Panorama capture stopped')
        else:
            response.success = False
            response.status_message = 'Panorama capture is already in the requested state.'
        return response

    def capture_frames(self):
        cap = None
        try:
            cap = cv2.VideoCapture(self.ip)
            if not cap.isOpened():
                self.get_logger().error('Failed to open video stream')
                return

            gps_data = self.get_gps_data()
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            session_directory = os.path.join(self.save_directory, f"{timestamp}_{gps_data}")
            os.makedirs(session_directory, exist_ok=True)

            start_time = datetime.now()
            while self.capture_event.is_set():
                ret, frame = cap.read()
                if ret:
                    with self.frame_count_lock:
                        self.frame_count += 1
                        frame_filename = os.path.join(session_directory, f"frame_{self.frame_count:04d}.jpg")
                    cv2.imwrite(frame_filename, frame)
                    self.get_logger().info(f'Saved frame {self.frame_count} to {frame_filename}')
                else:
                    self.get_logger().warn("Failed to capture frame")

                if datetime.now() - start_time > self.timeout_duration:
                    self.get_logger().info('Capture session timed out')
                    break

                rclpy.spin_once(self, timeout_sec=0.1)

        except Exception as e:
            self.get_logger().error(f"An error occurred during frame capture: {str(e)}")
        finally:
            if cap is not None and cap.isOpened():
                cap.release()
            self.capture_event.clear()
            self.get_logger().info(f"Capture session ended. Total frames captured: {self.frame_count}")

    def take_photo(self):
        try:
            cap = cv2.VideoCapture(self.ip)
            if not cap.isOpened():
                self.get_logger().error('Failed to open video stream')
                return False

            gps_data = self.get_gps_data()
            ret, frame = cap.read()
            if ret:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                photo_filename = os.path.join(self.save_directory, f"photo_{timestamp}_{gps_data}.jpg")
                cv2.imwrite(photo_filename, frame)
                self.get_logger().info(f'Took photo and saved to {photo_filename}')
                cap.release()
                return True
            else:
                self.get_logger().warn("Failed to capture photo")
                cap.release()
                return False
        except Exception as e:
            self.get_logger().error(f"An error occurred while taking a photo: {str(e)}")
            return False
    
    from sensor_msgs.msg import NavSatFix

    def get_gps_data(self):
       
        self.gps_data = None
        self.gps_data_received = False

       
        def callback(msg):
            self.gps_data = msg
            self.gps_data_received = True

       
        self.create_subscription(GpsPosition, '/rover/gps/position', callback, 10)

      
        timeout = rclpy.duration.Duration(seconds=5)  
        start_time = self.get_clock().now()

        while not self.gps_data_received and (self.get_clock().now() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.gps_data:
            return self.gps_data
        else:
            self.get_logger().warn("No GPS data received within the timeout period or topic does not exist.")
            return None



def main():
    rclpy.init()
    panorama_node = PanoramaService()
    rclpy.spin(panorama_node)
    panorama_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
