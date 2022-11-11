from __future__ import (print_function, absolute_import, division, unicode_literals)

import os
import rospkg
import rospy
from rospkg import RosPack
from python_qt_binding import loadUi
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QObject
from PyQt5.QtGui import QKeySequence
from PyQt5.QtWidgets import QShortcut, QSlider, QLCDNumber, QLabel
from rovus_bras.msg import feedback
from rovus_bras.msg import arm_gui_cmd

JOINT_NUMBER = 4

style_default = ""
style_Selected = "color: white; background-color: green"
style_Disable = "color: white; background-color: grey"
style_Limiting = "color: black; background-color: yellow"

class RoverArmGuiWidget(QtWidgets.QWidget):

    arm_gui_cmd_msg = arm_gui_cmd()

    def __init__(self):        
        super(RoverArmGuiWidget, self).__init__()

        ui_file = os.path.join(rospkg.RosPack().get_path('rover_arm_gui'), 'resource', 'rover_arm_gui.ui')
        loadUi(ui_file, self)
        self.setObjectName('RoverArmGuiWidget')

        self.is_active = False
        self.feedback_sub = rospy.Subscriber('rovus_bras_feedback', feedback, self.feedback_callback)
        self.pub_arm_gui_cmd = rospy.Publisher('arm_gui_cmd', arm_gui_cmd, queue_size=1)

        self.currSpeeds = [self.j1_currSpeed, self.j2_currSpeed, self.j3_currSpeed, self.j4_currSpeed]
        self.currAngles = [self.j1_currAngle, self.j2_currAngle, self.j3_currAngle, self.j4_currAngle]
        self.speedLabels = [self.j1_speedLabel, self.j2_speedLabel, self.j3_speedLabel, self.j4_speedLabel]

        self.j1_enable.pressed.connect(self.j1_enable_pressed_callback)
        self.j1_enable.released.connect(self.j1_enable_released_callback)
        self.j2_enable.pressed.connect(self.j2_enable_pressed_callback)
        self.j2_enable.released.connect(self.j2_enable_released_callback)
        self.j3_enable.pressed.connect(self.j3_enable_pressed_callback)
        self.j3_enable.released.connect(self.j3_enable_released_callback)
        self.j4_enable.pressed.connect(self.j4_enable_pressed_callback)
        self.j4_enable.released.connect(self.j4_enable_released_callback)


    def feedback_callback(self, data):
        #________________________________________
        #Angles LCD displays
        for i in range(len(self.currAngles)):
            self.currAngles[i].display(int(data.angles[i]))

        #________________________________________
        #Speed LCD displays    
        for i in range(len(self.currSpeeds)):
            self.currSpeeds[i].display(int(data.vitesses[i]))

        #________________________________________
        #State
        self.currState.setText("Running")
        
        #________________________________________
        #Control Mode
        if data.ctrl_mode:
            self.jogMode.setText("Joint")
        else:
            self.jogMode.setText("Cartesian")

        #________________________________________
        #Singular Matrix
        if data.singular_matrix:
            self.singularMatrix.setText("Singular Matrix --> Jog in joint")
        else:
            self.singularMatrix.setText("")

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
        #Limiting
        speedLabelsStyle = []
        for i in range(len(self.speedLabels)):
            speedLabelsStyle.append(style_default)

        if data.limiteur:
            for i in range(len(self.speedLabels)):
                if abs(int(data.vitesses[i])) == 20:
                    speedLabelsStyle[i] = style_Limiting

        for i in range(len(self.speedLabels)):
            self.speedLabels[i].setStyleSheet(speedLabelsStyle[i])
        
        #________________________________________
        #Speed Multiplicator
        self.currSpeedMultiplicator.display(data.speed_multiplier)

        self.publish_command();

    def publish_command(self):
        self.pub_arm_gui_cmd.publish(self.arm_gui_cmd_msg)

    def j1_enable_pressed_callback(self):
        self.arm_gui_cmd_msg.enable[0] = 0;
    
    def j1_enable_released_callback(self):
        self.arm_gui_cmd_msg.enable[0] = 1;

    def j2_enable_pressed_callback(self):
        self.arm_gui_cmd_msg.enable[1] = 0;
    
    def j2_enable_released_callback(self):
        self.arm_gui_cmd_msg.enable[1] = 1;

    def j3_enable_pressed_callback(self):
        self.arm_gui_cmd_msg.enable[2] = 0;
    
    def j3_enable_released_callback(self):
        self.arm_gui_cmd_msg.enable[2] = 1;

    def j4_enable_pressed_callback(self):
        self.arm_gui_cmd_msg.enable[3] = 0;
    
    def j4_enable_released_callback(self):
        self.arm_gui_cmd_msg.enable[3] = 1;






















    