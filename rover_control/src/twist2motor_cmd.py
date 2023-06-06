#!/usr/bin/env python

# -*- coding: utf-8 -*-

# Created on Feb 8 2021
# @author: Simon Chamorro           simon.chamorro@usherbrooke.ca
# @maintainer: Philippe Warren      philippe.warren@usherbrooke.ca

"""
@package rover_control

------------------------------------

ROS Node to go from a Twist to commands for the 4 robot motors
with watchdog security

"""

from __future__ import print_function, unicode_literals, division, absolute_import

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from ros_talon.msg import cmd
from std_msgs.msg import Empty

WATCHDOG_REFRESH_FREQUENCY = 2 #Hz

class LowLevelControlNode(object):
    def __init__(self):
        """Node class to control the motors"""
        rospy.init_node("twist2cmd", anonymous=False)
        rospy.on_shutdown(self.on_shutdown)

        # Getting params
        self._with_watchdog = rospy.get_param("~joy_with_watchdog", True)
        
        #flags
        self._watchdog_alive = False
        self._com_is_alive = False

        self.l_cmd = 0
        self.r_cmd = 0

        # Subscriber
        self.twist_sub = rospy.Subscriber("/cmd_vel", Twist, self.twist_callback)
        self.sub_watchdog = rospy.Subscriber("/base_heartbeat", Empty, self.watchdog_cb, queue_size=1)

        # Init publishers
        self.m1_pub = rospy.Publisher(
            '/ros_talon1/cmd', cmd, queue_size=10)  # Front left
        self.m2_pub = rospy.Publisher(
            '/ros_talon2/cmd', cmd, queue_size=10)  # Front right
        self.m3_pub = rospy.Publisher(
            '/ros_talon3/cmd', cmd, queue_size=10)  # Rear left
        self.m4_pub = rospy.Publisher(
            '/ros_talon4/cmd', cmd, queue_size=10)  # Rear right

        self.linear_factor_percentage = 100
        self.angular_factor_percentage = 30
        self.angular_gain = -4
        self.linear_gain = 0.8

        rospy.sleep(1)

        # Init command loop
        self.timer_send_cmd = rospy.Timer(rospy.Duration(1/50), self.send_cmd_callback)
        self.timer_reset_watchdog = rospy.Timer(rospy.Duration(WATCHDOG_REFRESH_FREQUENCY), self.reset_watchdog)

    def run(self):
        rospy.spin()

    def twist_callback(self, msg):
        """
        Callback when twist is received
        ----------
        Parameters
        ----------
        msg: Twist
            Twist message
        """
        self.compute_commands(msg)

    def send_cmd_callback(self, evt):
        """
        Send commands to motors timer
        """
        self.send_cmd()

    def compute_commands(self, msg):
        """
        Compute commands from twist message
        ----------
        Parameters
        ----------
        msg: Twist
            Twist message
        """
        linear_part = -msg.linear.x * self.linear_factor_percentage
        angular_part = msg.angular.z * self.angular_factor_percentage * self.angular_gain

        self.l_cmd = self.linear_gain * (-linear_part - angular_part)
        self.r_cmd = self.linear_gain * (linear_part - angular_part)

    def send_cmd(self):
        """
        Publishes commands
        """
        msgL = cmd()
        msgR = cmd()

        if (self._watchdog_alive):
            #Need to define cmd_mode
            msgL.cmd = float(self.l_cmd)
            msgL.cmd_mode = (int(0))

            msgR.cmd = float(self.r_cmd)
            msgR.cmd_mode = int(0)

        # Left wheels
        self.m1_pub.publish(msgL)
        self.m3_pub.publish(msgL)

        # Right wheels
        self.m2_pub.publish(msgR)
        self.m4_pub.publish(msgR)

    def watchdog_cb(self, msg):
        """
        Set watchdog bool according to heartbeat if this isn't called faster 
        then the heartbeat check, motors will only receive zeros
        """
        if self._with_watchdog:
            self._com_is_alive = True

    def reset_watchdog(self, none_arg_need_for_some_reason):
        if self._com_is_alive:
            self._watchdog_alive = True
        else:
            self._watchdog_alive = False
        self._com_is_alive = False


    def on_shutdown(self):
        """
        Set commands to 0 at shutdown
        """
        self.l_cmd = 0
        self.r_cmd = 0
        self.send_cmd()


if __name__ == "__main__":
    try:
        node = LowLevelControlNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
