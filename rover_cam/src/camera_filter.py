#!/usr/bin/env python

#Python libs
import sys, time

#Ros libs
import rospy

from cv_bridge import CvBridge

#numpy lib
import numpy as np

#openCV
import cv2

#Ros messages
from sensor_msgs.msg import Image, CompressedImage

#Services
from rover_cam.srv import gray_filter, gray_filterResponse, size_filter, size_filterResponse, fps_filter, fps_filterResponse


class CameraFilter:
    def __init__(self):
        #start node 'camera_filter'
        rospy.init_node('camera_filter', anonymous=True)

        #Global variables for services
        self.gray_filter_activated = rospy.get_param("~gray_filter_activated", False)
        self.size_filter_activated = rospy.get_param("~size_filter_activated", False)
        self.size_sampling_ratio = rospy.get_param("~size_sampling_ratio", 1.0)
        self.fps_filter_activated = rospy.get_param("~fps_filter_activated", False)
        self.fps = rospy.get_param("~fps", 30)

        #Start services
        s1 = rospy.Service('~gray_filter', gray_filter, self.handle_gray_filter) 
        s2 = rospy.Service('~size_filter', size_filter, self.handle_size_filter)
        s3 = rospy.Service('~fps_filter', fps_filter, self.handle_fps_filter)

        #initialize publishers
        self.filter_pub = rospy.Publisher('image_filtered', Image, queue_size=1)
        self.filter_pub_compressed = rospy.Publisher(self.filter_pub.name + '/compressed', CompressedImage, queue_size=1) 

        #subscribed to topic '/camera1/image_raw'
        self.camera_sub = rospy.Subscriber('/image', Image, self.imageCB, queue_size=1, buff_size=2**24)
       
        self.bridge = CvBridge()  

    def imageCB(self, ros_data):
        
        #limiter fps
        if self.fps_filter_activated == True:
            rate =rospy.Rate(self.fps)
        else:
            rate = rospy.Rate(30)
        
        #ouvrir l'image en couleur
        cv_image = self.bridge.imgmsg_to_cv2(ros_data, 'bgr8')

        #logique de transformation:

        #reduire la taille de l'image
        if self.size_filter_activated == True:
            cv_image = cv2.resize(cv_image, (0,0), fx=self.size_sampling_ratio, fy=self.size_sampling_ratio)

        #Convertir l'image en gray
        if self.gray_filter_activated == True:
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            data = self.bridge.cv2_to_imgmsg(cv_image, 'mono8')
            data_compressed = self.bridge.cv2_to_compressed_imgmsg(cv_image,'jpg')

        else:
            data = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
            self.filter_pub.publish(data)
            data_compressed = self.bridge.cv2_to_compressed_imgmsg(cv_image,'jpg')

        data.header.frame_id = ros_data.header.frame_id
        data_compressed.header.frame_id = ros_data.header.frame_id

        self.filter_pub.publish(data)
        self.filter_pub_compressed.publish(data_compressed)
        
        rate.sleep()

#Define services functions

    def handle_gray_filter(self, req):
        
        self.gray_filter_activated = req.active

        if self.gray_filter_activated == True :
            
            resp1 = gray_filterResponse(True)
            return resp1.result
        else :
            
            resp1 = gray_filterResponse(False)
            return resp1.result

    def handle_size_filter(self, req):

        if req.active == True:
            
            self.size_filter_activated = req.active
            self.size_sampling_ratio = req.size

            if self.size_sampling_ratio == req.size :
                return size_filterResponse(True)
            else :
                return size_filterResponse(False)
        else :
            self.size_filter_activated = req.active
            return size_filterResponse(False)

    def handle_fps_filter(self, req):

        if req.active == True: 
            self.fps_filter_activated = req.active
            self.fps = req.fps

            if self.fps == req.fps:
                return fps_filterResponse(True)
            else :
                return fps_filterResponse(False)
        else:
            self.fps_filter_activated = req.active
            return fps_filterResponse(False)


def main(args):
    node = CameraFilter()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:  
        pass     


if __name__ == '__main__':
    main(sys.argv)