#!/usr/bin/env python

# -*- coding: utf-8 -*-

# Created on Feb 8 2021
# @author: Simon Chamorro           simon.chamorro@usherbrooke.ca
# @maintainer: Philippe Warren      philippe.warren@usherbrooke.ca

"""
@package rover_control

------------------------------------

ROS Node to go from a Twist to commands for the 4 robot motors

"""

from __future__ import (print_function, unicode_literals,
                        division, absolute_import)

import rospy
import numpy as np
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist


class LowLevelControlNode(object):
    def __init__(self):
        """Node class to control the motors"""
        rospy.init_node('twist2cmd', anonymous=False)
        rospy.on_shutdown(self.on_shutdown)

        self.l_cmd = 0
        self.r_cmd = 0

        # Init publishers
        self.m1_pub = rospy.Publisher(
            '/ros_talon1/motor_percent', Int32, queue_size=10)  # Front left
        self.m2_pub = rospy.Publisher(
            '/ros_talon2/motor_percent', Int32, queue_size=10)  # Front right
        self.m3_pub = rospy.Publisher(
            '/ros_talon3/motor_percent', Int32, queue_size=10)  # Rear left
        self.m4_pub = rospy.Publisher(
            '/ros_talon4/motor_percent', Int32, queue_size=10)  # Rear right

        self.linear_factor_percentage = 100
        self.angular_factor_percentage = 30
        self.angular_gain = -4

        # Subscribe to joystick
        self.twist_sub = rospy.Subscriber(
            '/cmd_vel', Twist, self.twist_callback)

        rospy.sleep(1)

        # Init command loop
        rospy.Timer(rospy.Duration(1/50), self.send_cmd_callback)

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

        self.l_cmd = 0.04 * (- linear_part - angular_part)
        self.r_cmd = 0.04 * (linear_part - angular_part)

    def send_cmd(self):
        """
        Publishes commands
        """
        # Left wheels
        self.m1_pub.publish(int(self.l_cmd))
        self.m3_pub.publish(int(self.l_cmd))

        # Right wheels
        self.m2_pub.publish(int(self.r_cmd))
        self.m4_pub.publish(int(self.r_cmd))

    def on_shutdown(self):
        """
        Set commands to 0 at shutdown
        """
        self.l_cmd = 0
        self.r_cmd = 0
        self.send_cmd()


if __name__ == '__main__':
    try:
        node = LowLevelControlNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
