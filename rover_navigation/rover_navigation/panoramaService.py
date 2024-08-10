#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.task import Future
from rover_msgs.srv import Panorama
import cv2
import os
from datetime import datetime, timedelta
import threading
from ament_index_python.packages import get_package_share_directory

class PanoramaService(Node):
    def __init__(self):
        super().__init__('panorama_node')
        self.srv = self.create_service(Panorama, 'control_panorama', self.handle_service)
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

    def handle_service(self, request, response):
        if request.photo:
            self.get_logger().info('Taking picture')
            response.success = self.take_photo()
            response.status_message = 'Photo taken.' if response.success else 'Failed to take photo.'
        elif request.start and not self.is_capturing:
            self.get_logger().info('Starting panorama capture...')
            self.is_capturing = True
            self.capture_thread = threading.Thread(target=self.capture_frames)
            self.capture_thread.start()
            response.success = True
            response.status_message = 'Panorama capture started.'
            self.get_logger().info('Panorama capture started')
        elif request.stop and self.is_capturing:
            self.get_logger().info('Stopping panorama capture...')
            self.is_capturing = False
            if self.capture_thread:
                self.capture_thread.join()
            response.success = True
            response.status_message = 'Panorama capture stopped.'
            self.get_logger().info('Panorama capture stopped')
        else:
            response.success = False
            response.status_message = 'Panorama capture is already in the requested state.'
        return response

    def capture_frames(self):
        try:
            cap = cv2.VideoCapture(self.ip)
            if not cap.isOpened():
                self.get_logger().error('Failed to open video stream')
                return

            ## TODO confirmer que les donnees gps sont bien integree dans le nom du dossier ##
            gps_data = self.get_gps_data()
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            session_directory = os.path.join(self.save_directory, timestamp, f"{gps_data}")
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

    def take_photo(self):
        try:
            cap = cv2.VideoCapture(self.ip)
            if not cap.isOpened():
                self.get_logger().error('Failed to open video stream')
                return False

            ## TODO confirmer que les donnees gps sont bien integree dans le nom de la photo ##
            gps_data = self.get_gps_data()
            ret, frame = cap.read()
            if ret:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                photo_filename = os.path.join(self.save_directory, f"photo_{timestamp}.jpg", f"{gps_data}")
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
    
    def get_gps_data(self):
        gps_data_future = Future()
        self.create_subscription(float, '/rover/gps/position', lambda msg: gps_data_future.set_result(msg), 10)
        rclpy.spin_until_future_complete(self, gps_data_future)
        if gps_data_future.result(): return gps_data_future.result()
        return ""

def main():
    rclpy.init()
    panorama_node = PanoramaService()
    rclpy.spin(panorama_node)
    panorama_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
