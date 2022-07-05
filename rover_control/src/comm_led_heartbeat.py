from __future__ import print_function, absolute_import, unicode_literals, division

import rospy
from std_msgs.msg import Empty
from std_srvs.srv import SetBool, SetBoolResponse


class CommLedHeartbeat:
    def __init__(self):
        self._heartbeat_sub = rospy.Subscriber("/heartbeat", Empty, self.heartbeat_cb, queue_size=1)
        self._led_service = rospy.ServiceProxy("/relay/r7_cmd", SetBool)

        self.timeout = rospy.Timer(rospy.Duration(2), self.timeout_cb, oneshot=True)
        self.state = False

    def timeout_cb(self, _):
        self._led_service.call(False)

    def heartbeat_cb(self, _):
        if self.state is False:
            try:
                self._led_service.call(True)
                self.state = True
            except rospy.ServiceException as e:
                rospy.logwarn("Could not call relay 7 service: {}".format(e))
        else:
            self.timeout.shutdown()
            self.timeout = rospy.Timer(rospy.Duration(2), self.timeout_cb, oneshot=True)
