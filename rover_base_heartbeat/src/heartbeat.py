#!/usr/bin/env python

from __future__ import print_function, unicode_literals, absolute_import, division

# Python libs
import sys, time

# ROS libs
import rospy

# Ros messages
from std_msgs.msg import Int16

# Services
from rover_base_heartbeat.srv import E_stop, E_stopResponse


class HeartBeat:
    def __init__(self):
        # Start node
        rospy.init_node("heartbeat_node", anonymous=False)

        # E-stop service
        E_stop_service = rospy.Service("~E_stop", E_stop, self.handle_e_stop)
        self.E_stop_status = False

        # Start publisher
        self.beat_pub = rospy.Publisher("heartbeat", Int16, queue_size=1)

    def heartbeatCB(self):
        # Frequence du heartbeat
        freq = rospy.Rate(20)

        # Etat du message
        self.state = 0

        # Publish if not e-stop
        if not self.E_stop_status:
            self.beat_pub.publish(self.state)

        freq.sleep()

    def handle_e_stop(self, req):
        self.E_stop_status = req.E_stop
        if self.E_stop_status == True:
            # resp = E_stopResponse(True)
            return E_stopResponse(True)
        else:
            # resp = E_stopResponse(False)
            return E_stopResponse(False)


if __name__ == "__main__":

    try:
        node = HeartBeat()
        while not rospy.is_shutdown():

            node.heartbeatCB()

    except rospy.ROSInterruptException:

        pass
