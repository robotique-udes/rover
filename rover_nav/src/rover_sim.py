#!/usr/bin/python

# Description: Node that simulates the rover's pose and how it reacts when it receives twist commands.
#
# Authors: Jeremie Bourque
#
# Date created: 16-12-2019
# Date last updated: 18-12-2019

import rospy
import tf
import math
from geopoint import distanceBetween2CoordsXY, TranslateCoordinate, Wgs84ToXY, XYToWgs84
from geometry_msgs.msg import PoseStamped, Twist


# Rovers current position in the wgs84 frame. Starting value is arbitrary.
currentLat = 45.37834388
currentLon = -71.92689071

# Rovers current pose in the map frame, initialized to 0 (aka to the position of local_xy_origin.
currentX = 0
currentY = 0
currentHeading = -0.5


def sendTransform():
    (currentX, currentY) = Wgs84ToXY(currentLat, currentLon)
    # Send transform of rover in the map frame
    br.sendTransform((currentX, currentY, 0),
                     tf.transformations.quaternion_from_euler(0, 0, currentHeading),
                     rospy.Time.now(),
                     'rover',
                     'map')


def handleCmdVel(data):
    global currentLon, currentLat, currentHeading, currentX, currentY

    # Limit rovers linear and angular speed
    if data.linear.x > 0.5:
        data.linear.x = 0.5
    if data.angular.z > 0.2:
        data.angular.z = 0.2

    # Increment the rovers position in the map frame according to the twist command
    currentX += data.linear.x*(math.cos(currentHeading))
    currentY += data.linear.x*(math.sin(currentHeading))
    currentHeading += data.angular.z

    # Convert the map coordinates back into wgs84 coordinates to simulate the gps data received from the new position.
    (currentLat, currentLon) = XYToWgs84(currentX, currentY)


if __name__ == '__main__':
    rospy.init_node('rover_sim')
    br = tf.TransformBroadcaster()
    rospy.Subscriber('nav_cmd', Twist, handleCmdVel)
    origin_pose = rospy.wait_for_message('local_xy_origin', PoseStamped)
    rospy.loginfo("rover_sim ready")
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        sendTransform()
        rate.sleep()
    rospy.spin()
