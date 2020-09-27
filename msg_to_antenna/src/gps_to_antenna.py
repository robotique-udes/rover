#!/usr/bin/env python

import rospy
import socket
import struct
import math
from sensor_msgs.msg import NavSatFix

class GPSToAntenna: 
    def __init__(self):
        self.host = rospy.get_param("host", "10.42.0.30")  # server hostname
        self.port = rospy.get_param("port", 65432)  # server port
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP socket

        self.navsat_fix_format = "2f"  # lat, lon
        self.sub = rospy.Subscriber("fix", NavSatFix, self.navsatFixCB)
        self.time_between_msgs = rospy.get_param("time_between_msgs", 1)

        self.lat = 999  # init to 999 for invalid value
        self.lon = 999  # init to 999 for invalid value

        print("Using host %s with port %d" % (self.host, self.port))
        self.loop()

    def loop(self):
        print("gps_to_antenna node ready")
        while not rospy.is_shutdown():
            print("Sending: lat=%f  lon=%f" % (self.lat, self.lon))
            msg = struct.pack(self.navsat_fix_format, self.lat, self.lon)
            self.s.sendto(msg, (self.host, self.port))
            rospy.sleep(self.time_between_msgs)

    def navsatFixCB(self, msg):
        if not math.isnan(msg.latitude):
            self.lat = msg.latitude
        else:
            self.lat = 999
        if not math.isnan(msg.longitude):
            self.lon = msg.longitude
        else:
            self.lon = 999
        print("Subscriber: lat=%f  lon=%f" % (msg.latitude, msg.longitude))


if __name__ == "__main__":
    rospy.init_node("gps_to_antenna")
    gpsToAntenna = GPSToAntenna()
    rospy.spin()
