#!/usr/bin/env python

import rospy
from rover_udes.msg import CamCommand
from geometry_msgs.msg import Twist

class Input:
    def __init__(self):
        self.state = False
        self.message = None
        self.time_of_last_msg = 0.0

class CmdMux:
    def __init__(self):        
        rospy.init_node("cmd_mux")

        # velocity topics
        self.gui_vel_sub = rospy.Subscriber('/gui_cmd_vel', Twist, self.gui_vel_callback)
        self.joystick_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.joystick_vel_callback)
        self.nav_vel_sub = rospy.Subscriber('/nav_cmd', Twist, self.nav_vel_callback)
        self.vel_pub = rospy.Publisher('mux_cmd_vel', Twist, queue_size=1)

        # ptu topics
        self.gui_ptu_sub = rospy.Subscriber('/gui_cmd_ptu', CamCommand, self.gui_ptu_callback)
        self.joystick_ptu_sub = rospy.Subscriber('/cmd_ptu', CamCommand, self.joystick_ptu_callback)
        self.ptu_pub = rospy.Publisher('mux_cmd_ptu', CamCommand, queue_size=1)

        # parameters
        self.timeout = rospy.Duration(rospy.get_param("~timeout", 1))

        # inputs
        self.gui_vel_input = Input()
        self.joystick_vel_input = Input()
        self.nav_vel_input = Input()
        self.gui_ptu_input = Input()
        self.joystick_ptu_input = Input()
        self.vel_inputs = [self.joystick_vel_input, self.gui_vel_input, self.nav_vel_input]  # velocity inputs in order of priority
        self.ptu_inputs = [self.joystick_ptu_input, self.gui_ptu_input]  # ptu inputs in order of priority

    def gui_vel_callback(self, msg): 
        self.gui_vel_input.message = msg
        if msg.linear.x != 0 or msg.angular.z != 0:  # empty message
            self.gui_vel_input.state = True
            self.gui_vel_input.time_of_last_msg = rospy.Time.now()

    def joystick_vel_callback(self, msg):
        self.joystick_vel_input.message = msg
        if msg.linear.x != 0 or msg.angular.z != 0:  # empty message
            self.joystick_vel_input.state = True
            self.joystick_vel_input.time_of_last_msg = rospy.Time.now()

    def nav_vel_callback(self, msg):
        self.nav_vel_input.message = msg
        if msg.linear.x != 0 or msg.angular.z != 0:  # empty message
            self.nav_vel_input.state = True
            self.nav_vel_input.time_of_last_msg = rospy.Time.now()

    def gui_ptu_callback(self, msg):
        self.gui_ptu_input.message = msg
        if msg.cam_horizontal != 0 or msg.cam_vertical != 0:  # empty message
            self.gui_ptu_input.state = True
            self.gui_ptu_input.time_of_last_msg = rospy.Time.now()

    def joystick_ptu_callback(self, msg):
        self.joystick_ptu_input.message = msg
        if msg.cam_horizontal != 0 or msg.cam_vertical != 0:  # empty message
            self.joystick_ptu_input.state = True
            self.joystick_ptu_input.time_of_last_msg = rospy.Time.now()
    
    def publish_cmd(self):
        #print("publish loop")
        for vel_input in self.vel_inputs:
            if vel_input.state:
                #print("Found state true")
                if rospy.Time.now() - vel_input.time_of_last_msg > self.timeout:
                    vel_input.state = False
                else:
                    #print("publishing vel msg")
                    self.vel_pub.publish(vel_input.message)
                    break
                
        for ptu_input in self.ptu_inputs:
            if ptu_input.state:
                #print("Found state true")
                if rospy.Time.now() - ptu_input.time_of_last_msg > self.timeout:
                    ptu_input.state = False
                else:
                    #print("publishing vel msg")
                    self.ptu_pub.publish(ptu_input.message)
                    break

    def run(self):
        r = rospy.Rate(100)
        while not rospy.is_shutdown():
            self.publish_cmd()
            r.sleep()


if __name__ == '__main__':
    try:
        mux = CmdMux()
        print("cmd_mux ready")
        mux.run()
    except rospy.ROSInterruptException:
        pass
