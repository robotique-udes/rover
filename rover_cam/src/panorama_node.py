#!/usr/bin/env python

"""
Stitches panorama from camera feed
"""

import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
import roslib
import cv2
import rospkg
import subprocess
from cv_bridge import CvBridge, CvBridgeError


class PanoramaNode:
    def __init__(self):
        rospy.init_node("panorama_node")
        rospy.on_shutdown(self.on_shutdown)
        print "Starting panorama node"

        self.take_pic = False
        self.frames = []
        self.bridge = CvBridge()

        self.img_sub = rospy.Subscriber('usb_cam_0/image_raw', Image, self.img_cb)
        self.pic_sub = rospy.Subscriber('take_photo', Bool, self.picture_cb)
        self.pano_sub = rospy.Subscriber('stitch_pano', Bool, self.pano_cb)

    def picture_cb(self, take_pic):
        self.take_pic = take_pic        

    def pano_cb(self, stitch_pano):
        print('taking panorama')
        rospack = rospkg.RosPack()
        path = rospack.get_path('rover_cam') + '/panos/'
        for idx, frame in enumerate(self.frames):       
            cv2_img = self.bridge.imgmsg_to_cv2(frame, "bgr8")
            filename = path + './input_imgs/img' + str(idx + 1) + '.jpg'
            cv2.imwrite(filename, cv2_img)
            print(filename)
            rospy.sleep(1)
        try:
            subprocess.call(path + "stitch.sh " + path, shell=True)
        except:
            print('panorama failed!')

    def img_cb(self, img):
        if self.take_pic:
            print('taking picture #' + str(len(self.frames) + 1))
            self.frames.append(img)
            if len(self.frames) > 3:
                self.frames = self.frames[-3:] 
            self.take_pic = False
        else:
            pass

    def on_shutdown(self):
        pass


if __name__ == '__main__':
    try:
        pano_node = PanoramaNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass