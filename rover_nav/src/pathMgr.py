#!/usr/bin/python

# Description: Contains two services:
#                (1): Parses gpsGoals.txt, creates an array of poses and publishes it as a Path msg.
#                (2): Publishes a specific waypoint from the pose array as a PoseStamped msg.
#
# Authors: Jeremie Bourque
#
# Date created: 27-10-2019
# Date last updated: 01-12-2019

# gpsGoals.txt example:
#   #Example comment
#   45.50049,-73.62600 #example comment
#   45.50149,-73.62600
#   45.50249,-73.62800
#   45.50349,-73.62900

import os
import rospy
from geopoint import Geopoint
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from rover_nav.srv import createPath, createPathResponse
from rover_nav.srv import setWaypoint, setWaypointResponse


# Set current working directory as the path of this file
abspath = os.path.abspath(__file__)
dname = os.path.dirname(abspath)
os.chdir(dname)
GPSGoalsFile = "../gpsGoals.txt"


# Initialize the node and services.
def pathMgr():
    rospy.init_node('pathMgr')
    s1 = rospy.Service('createPath', createPath, handleCreatePath)
    s2 = rospy.Service('setWaypoint', setWaypoint, handleSetWaypoint)
    rospy.loginfo("PathMgr ready")
    rospy.spin()
     
# Handle createPath service requests
def handleCreatePath(req):
    rospy.loginfo("Creating path")
    path = Path()
    markerarray = MarkerArray()
    path.header.frame_id = "wgs84"
    path.header.stamp = rospy.Time.now()
    geopoints = parseGPSGoals()
    count = -1
    for gp in geopoints:
        markerarray.markers.append(gp.marker("wgs84", count, Marker().TEXT_VIEW_FACING, [1,1,0]))
        path.poses.append(gp.poseStamped("wgs84"))
        count -= 1
    pubPath.publish(path)
    pubPathMarkers.publish(deleteAllMarkerArrays())
    pubPathMarkers.publish(markerarray)
    return "Parsing completed"

# Handle setWaypoint service requests
def handleSetWaypoint(req):
    rospy.loginfo("Setting waypoint #%d as goal" % (req.waypointNumber))
    geopoints = parseGPSGoals()
    if len(geopoints) == 0:  # No path published, return error
        rospy.logwarn("Failed to create path, there are no points in the txt file")
        return "Failed, path empty"
    try:
        pubWaypoint.publish(geopoints[req.waypointNumber-1].poseStamped("wgs84"))
        pubMarker.publish(deleteAllMarkers())
        pubMarker.publish(geopoints[req.waypointNumber - 1].marker("wgs84", req.waypointNumber, Marker().SPHERE, [0,1,0]))
    except IndexError:
        rospy.logwarn("Waypoint number out of bounds")
    rospy.loginfo(str(geopoints[req.waypointNumber - 1]) + "waypoint set")
    return "Success"

# Parses the GPS goals text file to get array of gps goals.
def parseGPSGoals():
    # TODO: As a user, I want to easily see the list of points along with their number
    count = 1
    geopoints = []
    with open(GPSGoalsFile) as goals:
        lines = goals.readlines()
        #print("--Waypoints--")
        for line in lines:
            # Remove comment part of the line.
            lineWithoutComment = line[0:line.find("#")]
            comment = line[line.find("#"):]
            comment = comment.rstrip()
            if len(lineWithoutComment) != 0:
                if lineWithoutComment[-1] == ' ':
                    lineWithoutComment = lineWithoutComment[:-1]  # Remove unwanted space at the end of the string
                coordinate = lineWithoutComment.split(",")
                try:         
                    lat = float(coordinate[0])
                    lon = float(coordinate[1])
                    gp = Geopoint()
                    gp.setLon(lon)
                    gp.setLat(lat)
                    description = str(count)
                    if len(comment) != 0:
                        description += ": " + comment[1:]
                    gp.setDescription(description)
                    #print(str(count) + " " + str(gp))
                    geopoints.append(gp)
                    count += 1
                except ValueError:
                    rospy.loginfo("Not a float, ignoring line.")
    return geopoints

# Creates marker message with the action to delete all markers.
def deleteAllMarkerArrays():
    markerarray = MarkerArray()
    marker = Marker()
    marker.action = 3
    markerarray.markers.append(marker)
    return markerarray

def deleteAllMarkers():
    marker = Marker()
    marker.action = 3
    return marker

    
if __name__ == '__main__':
    pubPath = rospy.Publisher('path_topic', Path, queue_size=10)
    pubWaypoint = rospy.Publisher('waypoint_topic', PoseStamped, queue_size=10)
    pubMarker = rospy.Publisher('marker_topic', Marker, queue_size=10)
    pubPathMarkers = rospy.Publisher('path_marker_topic', MarkerArray, queue_size=10)
    pathMgr()

