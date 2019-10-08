#!/usr/bin/env python
import roslib
import rospy
from rover_control.msg import Command
from std_msgs.msg import Float32, Int32
from sensor_msgs.msg import Joy


class CmdMux:
    def __init__(self):
        # By default, commands come from joystick
        self.gui_control_on = False
        self.right_cmd = 0.0
        self.left_cmd = 0.0
        
        rospy.init_node("cmd_mux")
        rospy.on_shutdown(self.on_shutdown)
        self.gui_sub = rospy.Subscriber('/gui_cmd', Command, self.gui_cmd_callback)
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)

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
            self.left_cmd = joy.axes[1]
            self.right_cmd = joy.axes[4] 
        else:
            self.left_cmd = 0
            self.right_cmd = 0 


    def publish_cmd(self, event):
        cmd = Command()
        cmd.is_active = True
        cmd.right = self.right_cmd
        cmd.left = self.left_cmd
        self.cmd_pub.publish(cmd)

    def on_shutdown(self):
        pass


if __name__ == '__main__':
    try:
        mux = CmdMux()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass