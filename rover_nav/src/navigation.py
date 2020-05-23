#!/usr/bin/python

# Description: This node generates Twist commands to guide the rover towards its goal. It needs a goal with wgs84
#              coordinates (lat/lon) and the origin of the map frame from the /local_xy_origin topic in order to work.
#              The origin is defined in the launch file and automatically published.
#
# Authors: Jeremie Bourque
#
# Date created: 16-12-2019
# Date last updated: 16-18-2019

import rospy
import math
import tf
from geopoint import distanceBetween2CoordsXY, Wgs84ToXY, distanceBetweenToPoints
from geometry_msgs.msg import PoseStamped, Twist

# Constants
GOAL_REACHED_DIST = 5
GOAL_NEAR_DIST = 10
ANGLE_TOLERANCE = math.radians(10)
MAX_CMD = 100
STOP_AND_TURN_ANGLE = math.radians(45)


def handleGoal(data):
    global origin_pose
    currentGoal = data
    # Convert goal from wgs84 frame to map frame.
    (currentGoal.pose.position.x, currentGoal.pose.position.y) = Wgs84ToXY(currentGoal.pose.position.y, currentGoal.pose.position.x)

    goalReached = False
    rate = rospy.Rate(10)
    while not goalReached:
        # TODO: implement way to abort
        # TODO: Would it be simpler to broadcast the goal as a tf frame and directly lookup transform between rover and goal instead of calculation the transform manually?
        # Get transform between map and rover in the map frame.
        try:
            (trans, rot) = listener.lookupTransform('map', 'rover', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        # Calculate the distance between the rover and the goal in the map frame
        distX = currentGoal.pose.position.x - trans[0]
        distY = currentGoal.pose.position.y - trans[1]
        theta = (tf.transformations.euler_from_quaternion(rot))[2]

        # Use the transformation matrix (calculated by hand) to convert the distance between the rover and goal in the map frame to the rover's frame
        x = (distX*math.cos(theta)) + (distY*math.sin(theta))
        y = (-distX*math.sin(theta)) + (distY*math.cos(theta))

        # Distance to goal and angle to goal
        distanceToGoal = math.sqrt(x ** 2 + y ** 2)
        angleToGoal = math.atan2(y, x)

        # Generate Twist commands
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = move(distanceToGoal, angleToGoal)
        nav_cmd.publish(cmd)

        # If the rover is close enough the the goal, stop navigation.
        rospy.logwarn("Distance to goal: %f" % distanceToGoal)
        if distanceToGoal < GOAL_REACHED_DIST:
            goalReached = True
            rospy.loginfo("Goal Reached! Stopping navigation")

        rate.sleep()
    rospy.loginfo("Navigation stopped")


# Given a distance to goal and an angle to goal, will return twist cmds using the most appropriate drive profile
def move(dist, angle):
    # TODO: implement gradual acceleration and deceleration
    if abs(angle) > STOP_AND_TURN_ANGLE:
        linear, angular = moveStopAndTurn(angle)
    else:
        linear, angular = moveContinuous(dist, angle)
    return linear, angular


# Given an angle, will return twist cmds to turn without any linear movement
def moveStopAndTurn(angle):
    rospy.logwarn("moveStopAndTurn")
    if angle > ANGLE_TOLERANCE:
        angular = 100  # Turn counter-clockwise
    elif angle < -ANGLE_TOLERANCE:
        angular = -100  # Turn clockwise
    linear = 0
    return linear, angular


# Given an angle and a distance to the goal, will return twist cmds to reach that goal
def moveContinuous(dist, angle):
    rospy.logwarn("moveContinuous")
    if dist > GOAL_REACHED_DIST:
        linear = 100  # Move forward
    if angle > ANGLE_TOLERANCE:
        angular = 100  # Turn counter-clockwise
    elif angle < -ANGLE_TOLERANCE:
        angular = -100  # Turn clockwise
    else:
        angular = 0
    return linear, angular


if __name__ == '__main__':
    rospy.init_node('navigation')
    br = tf.TransformBroadcaster()
    origin_pose = rospy.wait_for_message('local_xy_origin', PoseStamped)
    listener = tf.TransformListener()
    nav_cmd = rospy.Publisher('nav_cmd', Twist, queue_size=10)
    rospy.loginfo("Navigation ready")
    rospy.Subscriber('waypoint_topic', PoseStamped, handleGoal)
    rospy.spin()
