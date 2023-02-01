#!/usr/bin/env python

import rospy
from robotnik_msgs.srv import set_digital_output
import time

def activateRelay10Hz():
    rospy.wait_for_service('/rly_08_node/set_digital_outputs')
    outputs = rospy.ServiceProxy('/rly_08_node/set_digital_outputs', set_digital_output)
    outputs(1, True)
    time.sleep(1)
    outputs(1, False)

    return


if __name__ == "__main__":
    activateRelay10Hz()
