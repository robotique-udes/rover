#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rover_msgs.msg import Aruco
from rover_msgs.srv import RtspStream
import cv2
import numpy as np
import time

class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.samplesize = 4
        self.ids_in_last_frames = []
        self.validated_ids = []
        self.list_frames = []
        self.frame = None
        self.cap = None
        self.rtsp_stream = None 
        
        self.last_processed_time = time.time()

        self.create_timer(1.0, self.timer_callback)
        self.streamID = self.create_service(RtspStream, 'stream_id', self.stream_CB)
        self.arucoPub = self.create_publisher(Aruco, '/rover/auxiliary/aruco', 10)

    def timer_callback(self):
        current_time = time.time()
        if current_time - self.last_processed_time < 1.0:
            return  

        aruco_msg = Aruco()
        aruco_msg.id = []  # Initialize with an empty list

        if self.cap is None or not self.cap.isOpened():
            self.get_logger().warn('No valid RTSP stream available')
            self.arucoPub.publish(aruco_msg)
            return

        if not self.add_frame():
            self.get_logger().warn('Failed to read frame from RTSP stream')
            self.arucoPub.publish(aruco_msg)
            return

        self.frame = self.convert_to_gray(self.frame)
        if self.frame is None:
            self.get_logger().error('Error converting frame to grayscale')
            self.arucoPub.publish(aruco_msg)
            return

        ids = self.scan_frame(self.frame)
        if ids is None:
            self.get_logger().info('No ArUCo markers in field of view')
        else:
            for i in range(len(ids)):
                id = np.squeeze(ids[i])
                if id >= len(self.ids_in_last_frames):
                    self.ids_in_last_frames.extend([0] * (id - len(self.ids_in_last_frames) + 1))
                self.ids_in_last_frames[id] += 1

            self.count_valid_ids()
            self.get_logger().info(f'ArUCo markers found: {self.validated_ids}')
            aruco_msg.id = self.validated_ids

        self.arucoPub.publish(aruco_msg)
        self.validated_ids = self.empty_list(self.validated_ids)

        if len(self.list_frames) > self.samplesize:
            self.delete_oldest_frame()

        self.last_processed_time = current_time
        
    def stream_CB(self, request, response):
        self.rtsp_stream = request.stream_id

        if self.cap is not None and self.cap.isOpened():
            self.cap.release()

        self.cap = cv2.VideoCapture(self.rtsp_stream)

        if not self.cap.isOpened():
            self.get_logger().warn(f'Failed to open RTSP stream: {self.rtsp_stream}')
            response.success = False
            return response

        ret, frame = self.cap.read()

        if not ret or frame is None:
            self.get_logger().warn(f'Failed to read frame from RTSP stream: {self.rtsp_stream}')
            self.cap.release()
            self.cap = None
            response.success = False
            return response

        self.frame = frame
        self.list_frames.insert(0, frame)
        response.success = True
        self.get_logger().info(f'Successfully connected to RTSP stream: {self.rtsp_stream}')
        return response
    
    def empty_list(self, list):
        list.clear()
        return list

    def convert_to_gray(self, frame):
        try:
            grey_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        except:
            self.get_logger().error('Error converting to grayscale')
            return None
        else:
            return grey_frame
    
    def add_frame(self):
        if self.cap is None or not self.cap.isOpened():
            self.get_logger().warn('No valid RTSP stream available')
            return False
    
        # Skip frames to get to the most recent one
        frames_to_skip = int(self.cap.get(cv2.CAP_PROP_FPS))
        for _ in range(frames_to_skip):
            self.cap.grab()
    
        ret, frame = self.cap.retrieve()
        if not ret or frame is None:
            self.get_logger().warn('Failed to read frame from RTSP stream')
            return False
    
        self.frame = frame
        self.list_frames.insert(0, frame)
        return True

    def delete_oldest_frame(self):
        if len(self.list_frames) > 0:
            ids = self.scan_frame(self.list_frames.pop())
            if ids is None:
                return 0
            for i in range(len(ids)):
                id = np.squeeze(ids[i])
                if id < len(self.ids_in_last_frames):
                    self.ids_in_last_frames[id] -= 1

    def count_valid_ids(self):
        for i in range(len(self.ids_in_last_frames)):
            if self.ids_in_last_frames[i] != 0:
                if (self.ids_in_last_frames[i] / self.samplesize) >= 1:
                    self.validated_ids.append(i)

    def scan_frame(self, frame):
        # Mirror the frame before scanning
        # mirrored_frame = cv2.flip(frame, 1)
        corners, ids, rejected = cv2.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.aruco_params)
        return ids
    
def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
