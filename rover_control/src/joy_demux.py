#! /usr/bin/env python
# -*- coding: utf8 -*-

from __future__ import print_function, division, unicode_literals, absolute_import

import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy
from std_srvs.srv import SetBool, SetBoolResponse
from threading import Lock


class JoyDemuxNode:
    def __init__(self):
        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joy_cb, queue_size=1)
        self.rover_joy_pub = rospy.Publisher("/rover_joy", Joy, queue_size=1)
        self.arm_joy_pub = rospy.Publisher("/arm_joy", Joy, queue_size=1)
        self.joy_state_pub = rospy.Publisher("/joy_state_is_arm", Bool, queue_size=1)

        self.set_arm_joy_srv = rospy.Service("set_arm_joy", SetBool, self.set_arm_joy_cb)

        self._is_arm_joy_lock = Lock()
        self._is_arm_joy = rospy.get_param("~start_as_arm_joy", False)
        self._toggle_arm_joy_btn = rospy.get_param("~toggle_arm_joy_btn", 3)
        self._allow_toggle = rospy.get_param("~allow_toggle", True)
        self._last_arm_joy_btn_state = False

        self.joy_state_pub_rate = rospy.Timer(rospy.Duration(1), self.publish_joy_state)

    def joy_cb(self, msg):
        with self._is_arm_joy_lock:
            if (
                self._allow_toggle
                and msg.buttons[self._toggle_arm_joy_btn] != self._last_arm_joy_btn_state
                and msg.buttons[self._toggle_arm_joy_btn] == 1
            ):
                self._is_arm_joy = not self._is_arm_joy
            self._last_arm_joy_btn_state = msg.buttons[self._toggle_arm_joy_btn]
            is_arm = self._is_arm_joy

        if is_arm:
            self.arm_joy_pub.publish(msg)
        else:
            self.rover_joy_pub.publish(msg)

    def set_arm_joy_cb(self, req):
        with self._is_arm_joy_lock:
            self._is_arm_joy = req.data
        return SetBoolResponse(success=True, message="")

    def publish_joy_state(self, _):
        with self._is_arm_joy_lock:
            self.joy_state_pub.publish(self._is_arm_joy)


if __name__ == "__main__":
    rospy.init_node("joy_demux")
    node = JoyDemuxNode()
    rospy.spin()
