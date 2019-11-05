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
from std_srvs.srv import SetBool, SetBoolResponse


class PanoramaNode:
    def __init__(self):
        rospy.init_node("panorama_node")
        rospy.on_shutdown(self.on_shutdown)
        print("Starting panorama node")
        print("Ready to be enabled")

        self.take_pic = False
        self.frames = []
        self.bridge = CvBridge()

        self.img_sub = None
        self.pic_sub = None
        self.pano_sub = None

        self.pano_service = rospy.Service('change_pano_state', SetBool, self.change_pano_state)

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
            print('panorama done!')
        except:
            print('panorama failed!')

    def change_pano_state(self, turn_on):
        response = SetBoolResponse()
        if turn_on.data:
            self.img_sub = rospy.Subscriber('usb_cam_0/image_raw', Image, self.img_cb)
            self.pic_sub = rospy.Subscriber('take_photo', Bool, self.picture_cb)
            self.pano_sub = rospy.Subscriber('stitch_pano', Bool, self.pano_cb)
            response.success = True
            print('Panorama node is enabled')
        elif self.img_sub is not None and self.pic_sub is not None \
        and self.pano_sub is not None:
            self.img_sub.unregister()
            self.pic_sub.unregister()
            self.pano_sub.unregister()
            response.success = False
            print('Panorama node is disabled')
        else:
            response.success = False
            print('Panorama node is disabled')
        return response

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