#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from rover_control_msgs.msg import motor_cmd

class TeleopJoystick:
    def __init__(self):
        rospy.init_node("teleop_joystick", anonymous=True)

        # Getting parameters initialized at the launch file
        self.enable_button = rospy.get_param("~enable_button", 4)
        self.turbo_button = rospy.get_param("~turbo_button", 5)
        self.axis_linear = rospy.get_param("~axis_linear", 1)
        self.axis_angular = rospy.get_param("~axis_angular", 0)

        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joyCB)

    def joyCB(self, data):  # calls both commands functions (for PTU and vehicle movements)
        something = 1

    def vehicleCMD(self):  # Publishes the vehicle movements commands to the cmd_vel topic
        msg: motor_cmd = motor_cmd()

if __name__ == "__main__":
    teleop_joystick = TeleopJoystick()
    rospy.loginfo("teleop_joystick ready")
    rospy.spin()
