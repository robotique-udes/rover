#! /usr/bin/env python
# -*- coding: utf8 -*-

from __future__ import print_function, division, unicode_literals, absolute_import

import rospy
from sensor_msgs.msg import Joy
from std_srvs.srv import SetBool, SetBoolResponse
from threading import Lock


class JoyDemuxNode:
    def __init__(self):
        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joy_cb, queue_size=1)
        self.rover_joy_pub = rospy.Publisher("/rover_joy", Joy, queue_size=1)
        self.arm_joy_pub = rospy.Publisher("/arm_joy", Joy, queue_size=1)

        self.set_arm_joy_srv = rospy.Service("set_arm_joy", SetBool, self.set_arm_joy_cb)

        self._is_arm_joy_lock = Lock()
        self._is_arm_joy = rospy.get_param("~start_as_arm_joy", False)

    def joy_cb(self, msg):
        with self._is_arm_joy_lock:
            is_arm = self._is_arm_joy

        if is_arm:
            self.arm_joy_pub.publish(msg)
        else:
            self.rover_joy_pub.publish(msg)

    def set_arm_joy_cb(self, req):
        with self._is_arm_joy_lock:
            self._is_arm_joy = req.data
        return SetBoolResponse(success=True, message="")
