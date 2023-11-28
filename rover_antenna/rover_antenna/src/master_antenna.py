#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Empty
from std_msgs.msg import String
from std_msgs.msg import Float32
import numpy

def goalHeading (latitudeRover:float, longitudeRover:float, latitudeAntenne:float, longitudeAntenne:float)->float:
    deltaLongitude = longitudeRover-longitudeAntenne
    return numpy.arctan2(numpy.sin(deltaLongitude)*numpy.cos(latitudeRover),numpy.cos(latitudeAntenne)*numpy.sin(latitudeRover)-numpy.sin(latitudeAntenne)*numpy.cos(latitudeRover)*numpy.cos(deltaLongitude))
    
def pubRotation(initialHeading: float,latitudeRover:float, longitudeRover:float, latitudeAntenne:float, longitudeAntenne:float):
    rotation = goalHeading(latitudeRover, longitudeRover, latitudeAntenne, longitudeAntenne)-initialHeading
    pub_rotation = rospy.Publisher("rotation",Float32,queue_size=1)
    rospy.init_node("master_antenna",anonymous=True)
    rate_de_la_rotation = rospy.Rate(1)
    while not rospy.is_shutdown() :
        pub_rotation.publish(rotation)
        rate_de_la_rotation.sleep()

def talker() :
    toggle_LED = rospy.Publisher("toggle_LED",Empty,queue_size=1)
    rospy.init_node("master_antenna",anonymous=True)
    rate_de_la_LED_June_Is_Better = rospy.Rate(1)
    while not rospy.is_shutdown() :
        toggle_LED.publish(Empty())
        rate_de_la_LED_June_Is_Better.sleep()


if __name__ == '__main__':
    try:
        pubRotation(0, 0.5, 0.6, 0.7, 0.8)
    except rospy.ROSInterruptException:
        pass

