#!/usr/bin/env python

import rospy
from copy import deepcopy
from math import sin, cos, pi 
from differential_drive.msg import VelocityTargets, Encoders
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster 
from tf.transformations import quaternion_from_euler

#TODO: wrap around should be handled in hardware interface and multiplier should be passed by message

class DifferentialDrive():
    def __init__(self):
        self.w = float(rospy.get_param("~base_width", 0.2))
        self.rate = rospy.get_param("~rate", 50)
        self.timeout_ticks = rospy.get_param("~timeout_ticks", 20)  # TODO: change to time or frequency instead of nb of ticks
        self.ticks_per_meter= float(rospy.get_param("~ticks_per_meter", 50))
        self.encoder_min = rospy.get_param("~encoder_min", -32768)
        self.encoder_max = rospy.get_param("~encoder_max", 32768)
        self.encoder_low_wrap = 0.3 + self.encoder_min
        self.encoder_high_wrap = 0.7 + self.encoder_min
        self.base_frame_id = rospy.get_param("~base_frame_id","base_link")
        self.odom_frame_id = rospy.get_param("~odom_frame_id", "odom")
        self.publish_tf = rospy.get_param("~publish_tf", True)

        self.vel_target = VelocityTargets()
        self.wheel_enc = Encoders()  # Wheel encoders with wrap around
        self.prev_wheel_enc = Encoders()  # Previous wheel encoders with wrap around
        self.prev_wheel_enc.left_encoder = None 
        self.prev_wheel_enc.right_encoder = None 

        self.odometry = Odometry()
        self.odometry.header.frame_id = self.odom_frame_id
        self.odometry.child_frame_id = self.base_frame_id
        self.odometry.pose.covariance = [1, 0, 0, 0, 0, 0,
                                         0, 1, 0, 0, 0, 0,
                                         0, 0, 1, 0, 0, 0,
                                         0, 0, 0, 1, 0, 0,
                                         0, 0, 0, 0, 1, 0,
                                         0, 0, 0, 0, 0, 1]
        self.odometry.twist.covariance = [1, 0, 0, 0, 0, 0,
                                         0, 1, 0, 0, 0, 0,
                                         0, 0, 1, 0, 0, 0,
                                         0, 0, 0, 1, 0, 0,
                                         0, 0, 0, 0, 1, 0,
                                         0, 0, 0, 0, 0, 1]
        self.position_x = 0
        self.position_y = 0
        self.orientation_z = 0

        self.pub_vel_target = rospy.Publisher("wheel_vel_target", VelocityTargets, queue_size=1)
        self.pub_odom = rospy.Publisher("odom", Odometry, queue_size=1)
        self.sub_cmd_vel = rospy.Subscriber("cmd_vel", Twist, self.cmdVelCB)
        self.sub_wheel_enc = rospy.Subscriber("wheel_enc", Encoders, self.encodersCB)
        self.odom_broadcaster = TransformBroadcaster()

        self.ticks_since_target = 0
        self.left_nb_of_wraps = 0
        self.right_nb_of_wraps = 0
        self.prev_time = None

    def cmdVelCB(self, msg):
        # Convert twist cmd to wheel target velocity
        if msg.angular.z == 0: 
            # Linear motion only
            self.vel_target.left_wheel_vel_target = msg.linear.x
            self.vel_target.right_wheel_vel_target = msg.linear.x
        elif msg.linear.x == 0:
            # Angular motion only
            self.vel_target.left_wheel_vel_target = -msg.angular.z * self.w / 2
            self.vel_target.right_wheel_vel_target = -self.vel_target.left_wheel_vel_target
        else:
            # Angular and linear motion
            radius = msg.linear.x / msg.angular.z
            self.vel_target.left_wheel_vel_target = msg.angular.z * (radius - (self.w / 2))
            self.vel_target.right_wheel_vel_target = msg.angular.z * (radius + (self.w / 2))
        self.ticks_since_target = 0

    def encodersCB(self, msg):
        self.wheel_enc.left_encoder = msg.left_encoder + msg.left_encoder_multiplier * (self.encoder_max - self.encoder_min)
        self.wheel_enc.right_encoder = msg.right_encoder + msg.right_encoder_multiplier * (self.encoder_max - self.encoder_min)
        current_time = rospy.Time.now()
        if self.prev_time != None:
            elapsed_time = (current_time - self.prev_time).to_sec()
            self.updateOdometry(elapsed_time)
        self.prev_time = current_time

	self.pub_odom.publish(self.odometry)
	if(self.publish_tf):
	    self.publishTf()

    def updateOdometry(self, elapsed_time):
        if self.prev_wheel_enc.left_encoder == None or self.prev_wheel_enc.right_encoder == None:
            d_left = 0
            d_right = 0
        else:
            d_left = (self.wheel_enc.left_encoder - self.prev_wheel_enc.left_encoder) / self.ticks_per_meter
            d_right = (self.wheel_enc.right_encoder - self.prev_wheel_enc.right_encoder) / self.ticks_per_meter
        self.prev_wheel_enc = deepcopy(self.wheel_enc)

        # Calculate distance and rotation traveled in the base frame in the x and y axis
        d_distance = (d_left + d_right) / 2
        if d_right - d_left == 0: 
            # Radius is infinite
            d_rotation = 0
            base_dx = d_distance 
            base_dy = 0
        else:
            # Radius is finite
            radius = (self.w / 2) * ((d_left + d_right) / (d_right - d_left))
            if radius == 0:
                d_rotation = (2 * d_right) / self.w
                base_dx = 0
                base_dy = 0
            else:
                d_rotation = d_distance / radius 
                base_dx = radius * sin(d_rotation)
                base_dy = -(radius * (cos(d_rotation) - 1))

        # Transform distances in the odom frame
        odom_dx = base_dx * cos(self.orientation_z) - base_dy * sin(self.orientation_z)
        odom_dy = base_dx * sin(self.orientation_z) + base_dy * cos(self.orientation_z)

        # Calculate velocities
        self.odometry.twist.twist.linear.x = d_distance / elapsed_time
        self.odometry.twist.twist.angular.z = d_rotation / elapsed_time

        # Calculate final pose of the robot
        self.odometry.pose.pose.position.x += odom_dx
        self.odometry.pose.pose.position.y += odom_dy
        self.orientation_z += d_rotation
        quaternion = quaternion_from_euler(0, 0, self.orientation_z)
        self.odometry.pose.pose.orientation.x = quaternion[0]
        self.odometry.pose.pose.orientation.y = quaternion[1]
        self.odometry.pose.pose.orientation.z = quaternion[2]
        self.odometry.pose.pose.orientation.w = quaternion[3]

    def publishTf(self):
        self.odom_broadcaster.sendTransform(
                (self.odometry.pose.pose.position.x, self.odometry.pose.pose.position.y, 0),
                (self.odometry.pose.pose.orientation.x, self.odometry.pose.pose.orientation.y,
                 self.odometry.pose.pose.orientation.z, self.odometry.pose.pose.orientation.w),
                rospy.Time.now(),
                self.base_frame_id,
                self.odom_frame_id
                )

    def loop(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            if self.ticks_since_target > self.timeout_ticks:
                self.vel_target.left_wheel_vel_target = 0
                self.vel_target.right_wheel_vel_target = 0
            self.odometry.header.stamp = rospy.Time.now()
            self.pub_vel_target.publish(self.vel_target)
            self.ticks_since_target += 1
            r.sleep()
            

if __name__ == "__main__":
    rospy.init_node("diff_drive")
    differential_drive = DifferentialDrive()
    rospy.loginfo("Differential drive ready")
    differential_drive.loop()
    
