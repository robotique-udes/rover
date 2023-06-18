#!/usr/bin/env python

import rospy
from robotnik_msgs.srv import set_digital_output
import time

def activateRelay1sec(relay):
    rospy.wait_for_service('/rly_08_node/set_digital_outputs')
    outputs = rospy.ServiceProxy('/rly_08_node/set_digital_outputs', set_digital_output)
    outputs(relay, True)
    time.sleep(1)
    outputs(relay, False)

    return


if __name__ == "__main__":
    activateRelay1sec(1)
