#!/usr/bin/env python
import roslib
import rospy
from rover_udes.msg import Command
from std_msgs.msg import Float32, Int32
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


MAX_VALUE = 100
MIN_VALUE = -100


class CmdMux:
    def __init__(self):
        # By default, commands come from joystick
        self.gui_control_on = False
        self.nav_control_on = False
        self.right_cmd = 0.0
        self.left_cmd = 0.0
        
        rospy.init_node("cmd_mux")
        rospy.on_shutdown(self.on_shutdown)
        self.gui_sub = rospy.Subscriber('/gui_cmd', Command, self.gui_cmd_callback)
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)
        self.nav_sub = rospy.Subscriber('/nav_cmd', Twist, self.nav_cmd_callback)

        self.cmd_pub = rospy.Publisher('mux_cmd', Command, queue_size=1)
        rospy.Timer(rospy.Duration(1.0 / 10.0), self.publish_cmd)

    def gui_cmd_callback(self, cmd):
        self.gui_control_on = cmd.is_active
        if self.gui_control_on:
            self.right_cmd = cmd.right
            self.left_cmd = cmd.left

    def joy_callback(self, joy):
        # Left bumper acts as dead-man switch
        if (joy.buttons[4] == 1) and not self.gui_control_on:
            self.left_cmd = joy.axes[1]*100
            self.right_cmd = joy.axes[4]*100
        else:
            self.left_cmd = 0
            self.right_cmd = 0

    def nav_cmd_callback(self, cmd):
        uncapped_left, uncapped_right = twistToTank(cmd.linear.x, cmd.angular.z)
        self.left_cmd, self.right_cmd = limitCmd(uncapped_left, uncapped_right)

    def publish_cmd(self, event):
        cmd = Command()
        cmd.is_active = True
        cmd.right = self.right_cmd
        cmd.left = self.left_cmd
        self.cmd_pub.publish(cmd)

    def on_shutdown(self):
        pass


# Convert twist commands to tank (left and right) command
def twistToTank(linear, angular):
    left_cmd = linear + angular
    right_cmd = linear - angular
    return left_cmd, right_cmd


# Limits right and left command from -100 to 100 while keeping the difference between the two proportional
def limitCmd(left, right):
    if not(MIN_VALUE < right < MAX_VALUE and MIN_VALUE < left < MAX_VALUE):
        if abs(right) >= abs(left):
            capped_right = 100
            capped_left = (left/abs(right)) * 100
        else:
            capped_left = 100
            capped_right = (right/abs(left)) * 100
        return capped_left, capped_right
    return left, right


if __name__ == '__main__':
    try:
        mux = CmdMux()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
