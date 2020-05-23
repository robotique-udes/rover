#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix

def simGPS():
	cur_lon = -73.5
	cur_lat = 45.5
	pub = rospy.Publisher('simulted_pos', NavSatFix, queue_size=10)
	rospy.init_node('simGPS', anonymous=False)
	rate = rospy.Rate(1) # 10hz
	msg = NavSatFix()
	while not rospy.is_shutdown():
		cur_lon += 0.001
		cur_lat += 0.001
		msg.longitude = cur_lon
		msg.latitude = cur_lat
		rospy.loginfo("Longitude = %f   Latitude = %f" % (msg.longitude, msg.latitude))
		pub.publish(msg)
		rate.sleep()

if __name__ == '__main__':
	try:
		simGPS()
	except rospy.ROSInterruptException:
		pass
