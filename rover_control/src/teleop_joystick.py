#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from rover_udes.msg import CamCommand

class TeleopJoystick():
    def __init__(self):
        rospy.init_node("teleop_joystick", anonymous = True)

        #Getting parameters initialized at the launch file
        self.enable_button = rospy.get_param("~enable_button", 4)
        self.enable_turbo_button = rospy.get_param("~enable_turbo_button", 5)
        self.linear_scaling = rospy.get_param("~scale_linear", 1.0)
        self.linear_scaling_turbo = rospy.get_param("~scale_linear_turbo", 4.0)
        self.angular_scaling = rospy.get_param("~scale_angular", 1.0)
        self.axis_linear = rospy.get_param("~axis_linear", 1)
        self.axis_angular = rospy.get_param("~axis_angular", 0)
        self.ptu_vertical_axis = rospy.get_param("~ptu_vertical_axis", 4)
        self.ptu_horizontal_axis = rospy.get_param("~ptu_horizontal_axis", 3)
        self.ptu_horizontal_scaling = rospy.get_param("~ptu_horizontal_scaling", 1)
        self.ptu_vertical_scaling = rospy.get_param("~ptu_vertical_scaling", 1)

        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joyCB)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=2)
        self.cmd_ptu_pub = rospy.Publisher('/cmd_ptu', CamCommand, queue_size=2)

    def joyCB(self, data): #calls both commands functions (for PTU and vehicle movements)
        self.vehicleCMD(data)
        self.ptuCMD(data)

    def vehicleCMD(self,data): #Publishes the vehicle movements commands to the cmd_vel topic
        twist = Twist()
        twist.linear.x = data.axes[self.axis_linear]
        twist.angular.z = data.axes[self.axis_angular]

        if data.buttons[self.enable_button] == 1: #Verify the dead man button is active (LB)
            twist.angular.z *= self.angular_scaling #Both the turbo mode and normal mode share the same angular speed

            if data.buttons[self.enable_turbo_button] == 1: #Verify if the turbo button is active (RB)
                twist.linear.x *= self.linear_scaling_turbo
                self.cmd_pub.publish(twist)
            else:
                twist.linear.x *= self.linear_scaling 
                self.cmd_pub.publish(twist)
        else:
            twist.angular.z = 0
            twist.linear.x = 0
            self.cmd_pub.publish(twist)

    def ptuCMD(self,data): #Publishes the pan and tilt unit commands to the cmd_panTilt topic
        cam_cmd = CamCommand()
        cam_cmd.mode = 1  # Velocity mode
        cam_cmd.cam_horizontal = data.axes[self.ptu_vertical_axis]
        cam_cmd.cam_vertical = data.axes[self.ptu_horizontal_axis]
        self.cmd_ptu_pub.publish(cam_cmd)

if __name__ == '__main__':
    teleop_joystick = TeleopJoystick()
    rospy.loginfo("teleop_joystick ready")
    rospy.spin()

    