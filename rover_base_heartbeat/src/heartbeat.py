#!/usr/bin/env python

#Python libs
import sys, time

#ROS libs
import roslib
import rospy
import rostopic

#Ros messages
from std_msgs.msg import Int16

class HeartBeat:
    def __init__(self):
        #Start node 
        rospy.init_node('heartbeat_node', anonymous=True)

        #Start publisher
        self.beat_pub = rospy.Publisher('heartbeat', Int16, queue_size=1)
        
    def heartbeatCB(self):
        #Frequence du heartbeat
        freq = rospy.Rate(2)

        #Etat du message
        self.state = 0

        #Publier l'etat
        self.beat_pub.publish(self.state)

        freq.sleep()
             
if __name__ == '__main__':
    
    try:
        while not rospy.is_shutdown():
            node = HeartBeat()
            node.heartbeatCB()
        
    except rospy.ROSInterruptException:  
        
        pass  
    
    