#!/usr/bin/env python
import rospy
import rostopic
from rostopic import ROSTopicHz
from diagnostic_updater import *
import diagnostic_msgs
from std_msgs.msg import Float32, Int32, Int16 
from geometry_msgs.msg import Twist


class FrequencyStatus(DiagnosticTask):
    """A diagnostic task that monitors the frequency of an event.

    This diagnostic task monitors the frequency of calls to its tick method,
    and creates corresponding diagnostics. It will report a warning if the
    frequency is outside acceptable bounds, and report an error if there have
    been no events in the latest window.
    """

    def __init__(self, name, params, topic, sensor_id):
        """Constructs a FrequencyStatus class with the given parameters."""
        DiagnosticTask.__init__(self, name)
        self.params = params
        self.lock = threading.Lock()
        self.clear()
        self.sub = rospy.Subscriber(topic, rospy.AnyMsg, self.tick)
        self.state = 2
        self.id = sensor_id

    def clear(self):
        """Resets the statistics."""
        with self.lock:
            self.count = 0
            curtime = rospy.Time.now()
            self.times = [curtime for i in range(self.params.window_size)]
            self.seq_nums = [0 for i in range(self.params.window_size)]
            self.hist_indx = 0

    def tick(self, msg):
        """Signals that an event has occurred."""
        with self.lock:
            self.count += 1

    def run(self, stat):
        with self.lock:
            curtime = rospy.Time.now()
            curseq = self.count
            events = curseq - self.seq_nums[self.hist_indx]
            window = (curtime - self.times[self.hist_indx]).to_sec() + 0.0001
            freq = events / window
            self.seq_nums[self.hist_indx] = curseq
            self.times[self.hist_indx] = curtime
            self.hist_indx = (self.hist_indx + 1) % self.params.window_size

            if events == 0:
                stat.summary(2, "No events recorded.")
                self.state = 2
            elif freq < self.params.freq_bound['min'] * (1 - self.params.tolerance):
                stat.summary(1, "Frequency too low.")
                self.state = 1
            elif self.params.freq_bound.has_key('max') and freq > self.params.freq_bound['max'] * (1 + self.params.tolerance):
                stat.summary(1, "Frequency too high.")
                self.state = 1
            else:
                stat.summary(0, "Desired frequency met")
                self.state = 0

            stat.add("Events in window", "%d" % events)
            stat.add("Events since startup", "%d" % self.count)
            stat.add("Duration of window (s)", "%f" % window)
            stat.add("Actual frequency (Hz)", "%f" % freq)
            if self.params.freq_bound.has_key('max') and self.params.freq_bound['min'] == self.params.freq_bound['max']:
                stat.add("Target frequency (Hz)", "%f" % self.params.freq_bound['min'])
            if self.params.freq_bound['min'] > 0:
                stat.add("Minimum acceptable frequency (Hz)", "%f" % (self.params.freq_bound['min'] * (1 - self.params.tolerance)))
            if self.params.freq_bound.has_key('max'):
                stat.add("Maximum acceptable frequency (Hz)", "%f" % (self.params.freq_bound['max'] * (1 + self.params.tolerance)))

        return stat

class Emergency_stop:

    def __init__(self, freq_check_list):
        self.freq_check = freq_check_list

        #Subscribed to move command topic
        self.Move_sub = rospy.Subscriber('/mux_cmd_vel', Twist, self.E_STOP_CB, queue_size=1 )
        self.Move_pub = rospy.Publisher('/watchdog/cmd_vel', Twist, queue_size=1)

    def E_STOP_CB(self, data):
        self.stop = 0
        
        for i in self.freq_check:
            if i.state != 0:
                self.stop = 1

        if self.stop == 0:
            self.Move_pub.publish(data)
        else :
            # Publish a 0 velocity message for safety
            self.Move_pub.publish(Twist())
        

if __name__ == '__main__':
    rospy.init_node("sensor_check")
    # Wait for the ROS clock to be initialized

    r = rospy.Rate(100)
    while not rospy.Time.now().to_sec() > 0:
        r.sleep
        
    nb_sensors = int(rospy.get_param("~nb_sensors", 0))
    updaters = []
    pkg_name = rospy.get_name()
    freq_check_list = []

    for i in range(1, nb_sensors + 1):
        name = ('~' + pkg_name + '/sensor_' + str(i))

        # sensor parameters
        path = str(rospy.get_param((name + '/name'), ''))
        sensor_id = str(rospy.get_param((name + '/sensor_id'), ''))
        topic = str(rospy.get_param((name + '/topic'), ''))
        min_freq = float(rospy.get_param((name + '/min_freq'), ''))
        max_freq = float(rospy.get_param((name + '/max_freq'), ''))

        updater = Updater()
        updater.setHardwareID(sensor_id)

        # Diagnostic tasks are added to the Updater. They will later be run when
        # the updater decides to update.
        param = FrequencyStatusParam({'min': min_freq, 'max': max_freq}, tolerance=0, window_size=20)
        freq_check = FrequencyStatus(path, param, topic, sensor_id)
        updater.add(freq_check)
        freq_check_list.append(freq_check)
        # If we know that the state of the node just changed, we can force an
        # immediate update.
        updater.force_update()
        updaters.append(updater)

    E_stop = Emergency_stop(freq_check_list)
    
    while not rospy.is_shutdown():
        rospy.sleep(0.1)
        # We can call updater.update whenever is convenient. It will take care
        # of rate-limiting the updates.
        for u in updaters:
             
            u.force_update()


