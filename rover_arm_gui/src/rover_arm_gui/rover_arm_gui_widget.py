from __future__ import (print_function, absolute_import, division, unicode_literals)

import os
import rospkg
import rospy
import roslaunch
import os
from rospkg import RosPack
from python_qt_binding import loadUi
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QObject
from PyQt5.QtGui import QKeySequence
from PyQt5.QtWidgets import QShortcut, QSlider, QLCDNumber, QLabel, QPushButton, QFrame
from rover_arm_msgs.msg import feedback
from rover_arm_msgs.msg import arm_gui_cmd
from std_srvs.srv import SetBool, SetBoolResponse
import rosservice
from threading import Lock

#Constantes
JOINT_NUMBER = 4
FEEDBACK_CALLBACK_FREQUENCY = 10 #Hz
STANDBY = 0
RUN = 1
CALIB = 2

#Styling "MACROS"
style_default = ""
style_Selected = "color: white; background-color: green"
style_Disable = "color: white; background-color: grey"
style_Limiting = "color: black; background-color: yellow"
style_Warn = "color: black; background-color: rgb(255, 124, 0);"


class RoverArmGuiWidget(QtWidgets.QWidget):
    j_enable_state: bool = [1, 1, 1, 1]
    launch_3d_view_flag: bool = 0
    launch_all_views_flag: bool = 0
    img_keybinding_state: bool = 0
    curr_state = RUN
    joy_demux_state: bool = 0
    cartesian_jog_mode_state: bool = 0

    arm_gui_cmd_msg = arm_gui_cmd()

    def __init__(self):
        
        self.lock = Lock()        
        super(RoverArmGuiWidget, self).__init__()

        ui_file = os.path.join(rospkg.RosPack().get_path('rover_arm_gui'), 'resource', 'rover_arm_gui.ui')
        loadUi(ui_file, self)
        self.setObjectName('RoverArmGuiWidget')

        self.launch: roslaunch.ROSLaunch = roslaunch.scriptapi.ROSLaunch()
        self.launch.start()

        self.lastTimeFeedback = rospy.Time.now()

        self.is_active = False
        self.feedback_sub = rospy.Subscriber('rover_arm_feedback', feedback, self.feedback_callback, queue_size=1)
        self.pub_arm_gui_cmd = rospy.Publisher('arm_gui_cmd', arm_gui_cmd, queue_size=1)
        self.set_arm_joy = rospy.ServiceProxy('set_arm_joy', SetBool)

        self.img_keybinding.hide()
        self.currSpeeds = [self.j1_currSpeed, self.j2_currSpeed, self.j3_currSpeed, self.j4_currSpeed]
        self.currAngles = [self.j1_currAngle, self.j2_currAngle, self.j3_currAngle, self.j4_currAngle]

        self.j1_enable_.released.connect(lambda: self.j_enable_released_callback(1))
        self.j2_enable_.released.connect(lambda: self.j_enable_released_callback(2))
        self.j3_enable_.released.connect(lambda : self.j_enable_released_callback(3))
        self.j4_enable_.released.connect(lambda : self.j_enable_released_callback(4))

        self.launch_3d_view.released.connect(self.launch_3d_view_released_callback)
        self.launch_all_views.released.connect(self.launch_all_views_released_callback)
        self.show_keybinding.released.connect(self.show_keybinding_callback)
        self.ctrl_joy_demux.released.connect(self.ctrl_joy_demux_callback)
        self.cartesian_jog_mode_.released.connect(self.cartesian_jog_mode_callback)
        self.state_standby.clicked.connect(self.state_standby_callback)
        self.state_run.clicked.connect(self.state_run_callback)
        self.state_calib.clicked.connect(self.state_calib_callback)

    def feedback_callback(self, data):
        if self.lastTimeFeedback + rospy.Duration(1.0/FEEDBACK_CALLBACK_FREQUENCY) > rospy.Time.now() :
            return

        #Testing
        #self.shadowButton.

        self.lastTimeFeedback = rospy.Time.now()
        with self.lock:
            #________________________________________
            #Angles LCD displays
            for i in range(len(self.currAngles)):
                self.currAngles[i].setValue(data.angles[i])
                #rospy.loginfo("Displaying angle")
            
            #________________________________________
            #Speed LCD displays    
            for i in range(len(self.currSpeeds)):
                self.currSpeeds[i].setValue(data.vitesses[i])

            #________________________________________
            #Singular Matrix
            if data.singular_matrix:
                self.singularMatrix.setText("Singular Matrix --> Jog in joint")
                self.singularMatrix.setStyleSheet(style_Warn)
            else:
                self.singularMatrix.setText("")
                self.singularMatrix.setStyleSheet(style_default)

            #________________________________________
            #Style init
            jointLabelsStyle = []
            for i in range(JOINT_NUMBER):
                jointLabelsStyle.append(style_default)



            #________________________________________
            #Selected join
            if data.ctrl_mode:
                for i in range(len(jointLabelsStyle)):
                    if data.current_joint == i+1:
                        jointLabelsStyle[i] = style_Selected

            #________________________________________
            #Enable
            for i in range(len(jointLabelsStyle)):
                if not data.enable[i]:
                    jointLabelsStyle[i]=style_Disable

            #________________________________________
            #Style update
            self.moteur1Label.setStyleSheet(jointLabelsStyle[0])
            self.moteur2Label.setStyleSheet(jointLabelsStyle[1])
            self.moteur3Label.setStyleSheet(jointLabelsStyle[2])
            self.moteur4Label.setStyleSheet(jointLabelsStyle[3])

            #________________________________________
            #Speed Multiplicator
            self.currSpeedMultiplicator.setValue(data.speed_multiplier)

        self.publish_command();

    def publish_command(self):
        with self.lock:
            i = 0
            for i in range(len(self.arm_gui_cmd_msg.enable)):
                self.arm_gui_cmd_msg.enable[i] = self.j_enable_state[i]

            self.arm_gui_cmd_msg.state = self.curr_state;
            self.arm_gui_cmd_msg.jog_is_cartesian = self.cartesian_jog_mode_state;
            
            self.pub_arm_gui_cmd.publish(self.arm_gui_cmd_msg)

    def j_enable_released_callback(self, jointNb):
        self.j_enable_state[jointNb-1] = not self.j_enable_state[jointNb-1]

    def launch_3d_view_released_callback(self):
        if not self.launch_3d_view_flag:
            self.launch_3d_view_flag = 1

            node_live3DView = roslaunch.core.Node("rover_arm", "liveGraph3DView.py")
            self.live3DView_process = self.launch.launch(node_live3DView)
            self.launch_3d_view.setText("Close 3D view")

        else:
            self.launch_3d_view_flag = 0
            self.live3DView_process.stop()
            self.launch_3d_view.setText("Launch 3D view")

    def launch_all_views_released_callback(self):
        if not self.launch_all_views_flag:
            self.launch_all_views_flag = 1

            node_liveAllViews = roslaunch.core.Node("rover_arm", "liveGraphAllViews.py")
            self.liveAllViews_process = self.launch.launch(node_liveAllViews)
            self.launch_all_views.setText("Close All views")

        else:
            self.launch_all_views_flag = 0
            self.liveAllViews_process.stop()
            self.launch_all_views.setText("Launch all views")

    def show_keybinding_callback(self):
        if self.img_keybinding_state:
            self.img_keybinding.hide()
            self.img_keybinding_state = 0
        else:
            self.img_keybinding_state = 1
            self.img_keybinding.show()

    def state_standby_callback(self):
        self.curr_state = STANDBY;
        self.state_standby.setStyleSheet(style_Selected)
        self.state_run.setStyleSheet(style_default)
        self.state_calib.setStyleSheet(style_default)
        self.currState.setText(" Standing by")

    def state_run_callback(self):
        self.curr_state = RUN;
        self.state_standby.setStyleSheet(style_default)
        self.state_run.setStyleSheet(style_Selected)
        self.state_calib.setStyleSheet(style_default)
        self.currState.setText(" Running")

    def state_calib_callback(self):
        self.curr_state = CALIB;
        self.state_standby.setStyleSheet(style_default)
        self.state_run.setStyleSheet(style_default)
        self.state_calib.setStyleSheet(style_Selected)
        self.currState.setText(" Calibration in progress")

    def ctrl_joy_demux_callback(self):
        if '/set_arm_joy' in rosservice.get_service_list():
            self.ctrl_joy_demux.setText("Joy to arm")

            rospy.wait_for_service('set_arm_joy')
            if self.joy_demux_state:
                if(self.set_arm_joy(False)):
                    self.joy_demux_state = False;
                    self.ctrl_joy_demux.setStyleSheet(style_default)
            else:
                if(self.set_arm_joy(True)):
                    self.joy_demux_state = True;
                    self.ctrl_joy_demux.setStyleSheet(style_Selected)
        else:
            rospy.logerr("Service 'set_arm_joy' not available")
            self.ctrl_joy_demux.setText("'set_arm_joy' not available")
            self.ctrl_joy_demux.setStyleSheet(style_Warn)

    def cartesian_jog_mode_callback(self):
        if (self.cartesian_jog_mode_state):
            self.cartesian_jog_mode_.setStyleSheet(style_default)
        else:
            self.cartesian_jog_mode_.setStyleSheet(style_Selected)

        self.cartesian_jog_mode_state = not self.cartesian_jog_mode_state
