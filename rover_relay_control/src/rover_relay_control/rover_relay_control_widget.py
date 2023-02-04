#!/usr/bin/env python

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
from std_srvs.srv import SetBool, SetBoolResponse
from robotnik_msgs.srv import set_digital_output
import rosservice

style_default = ""
style_Selected = "color: white; background-color: green"
style_Disable = "color: white; background-color: grey"
style_Limiting = "color: black; background-color: yellow"
activate1 = True


class RoverRelayControlWidget(QtWidgets.QWidget):

    
    LUMIERES = [1, 2]
    INDICATEURS = [3, 4]
    LUMIERE1 = [1]
    LUMIERE2 = [2]
    RELAIS3 = [3]
    RELAIS4 = [4]
    RELAIS5 = [5]
    RELAIS6 = [6]
    RELAIS7 = [7]
    RELAIS8 = [8]
    TOUS = [0]
    activate = [True,True,True,True,True,True,True,True]
    states_lumieres = [activate[0], activate[1]]
    states_indicateurs = [activate[2], activate[3]]

    def __init__(self):        
        super(RoverRelayControlWidget, self).__init__()

        ui_file = os.path.join(rospkg.RosPack().get_path('rover_relay_control'), 'resource', 'rover_relay_control.ui')
        loadUi(ui_file, self)
        self.setObjectName('RoverRelayControlWidget')

        self.launch: roslaunch.ROSLaunch = roslaunch.scriptapi.ROSLaunch()
        self.launch.start()

        self.is_active = False

        # self.bouton_relay1.released.connect(lambda : self.activate_relay_callback(1,self.activate[0]))
        # self.bouton_relay2.released.connect(lambda : self.activate_relay_callback(2,self.activate[1]))
        # self.bouton_relay3.released.connect(lambda : self.activate_relay_callback(3,self.activate[2]))
        # self.bouton_relay4.released.connect(lambda : self.activate_relay_callback(4,self.activate[3]))
        # self.bouton_relay5.released.connect(lambda : self.activate_relay_callback(5,self.activate[4]))
        # self.bouton_relay6.released.connect(lambda : self.activate_relay_callback(6,self.activate[5]))
        # self.bouton_relay7.released.connect(lambda : self.activate_relay_callback(7,self.activate[6]))
        # self.bouton_relay8.released.connect(lambda : self.activate_relay_callback(8,self.activate[7]))
        # self.bouton_desactiver_relay.released.connect(lambda : self.activate_relay_callback(0,False))
        self.bouton_relay1.released.connect(lambda : self.control_relay_group_callback(self.LUMIERE1, [self.activate[self.LUMIERE1[0]-1]]))
        self.bouton_relay2.released.connect(lambda : self.control_relay_group_callback(self.LUMIERE2, [self.activate[self.LUMIERE2[0]-1]]))
        self.bouton_relay3.released.connect(lambda : self.control_relay_group_callback(self.RELAIS3, [self.activate[self.RELAIS3[0]-1]]))
        self.bouton_relay4.released.connect(lambda : self.control_relay_group_callback(self.RELAIS4, [self.activate[self.RELAIS4[0]-1]]))
        self.bouton_relay5.released.connect(lambda : self.control_relay_group_callback(self.RELAIS5, [self.activate[self.RELAIS5[0]-1]]))
        self.bouton_relay6.released.connect(lambda : self.control_relay_group_callback(self.RELAIS6, [self.activate[self.RELAIS6[0]-1]]))
        self.bouton_relay7.released.connect(lambda : self.control_relay_group_callback(self.RELAIS7, [self.activate[self.RELAIS7[0]-1]]))
        self.bouton_relay8.released.connect(lambda : self.control_relay_group_callback(self.RELAIS8, [self.activate[self.RELAIS8[0]-1]]))
        self.bouton_desactiver_relay.released.connect(lambda : self.control_relay_group_callback(self.TOUS, [False]))
        self.bouton_activer_lumieres.released.connect(lambda : self.control_relay_group_callback(self.LUMIERES, self.states_lumieres))        


    def activate_relay_callback(self, relay, state):
        rospy.wait_for_service('/rly_08_node/set_digital_outputs')
        outputs = rospy.ServiceProxy('/rly_08_node/set_digital_outputs', set_digital_output)
        if state:
            outputs(relay, state)
            self.activate[relay-1] = False
        else:
            outputs(relay, state)
            self.activate[relay-1] = True
        
        if relay == 0:
            self.activate = [True,True,True,True,True,True,True,True]

    def control_relay_group_callback(self, relays, states):
        rospy.wait_for_service('/rly_08_node/set_digital_outputs')
        outputs = rospy.ServiceProxy('/rly_08_node/set_digital_outputs', set_digital_output)
        
        for i in range(len(relays)):
            if states[i]:
                outputs(relays[i], states[i])
                self.activate[relays[i]-1] = False
            else:
                outputs(relays[i], states[i])
                self.activate[relays[i]-1] = True
        
        self.states_lumieres = [self.activate[0], self.activate[1]]

        if relays == self.TOUS:
            self.activate = [True,True,True,True,True,True,True,True]

        if self.states_lumieres != [True,True]:
            self.bouton_activer_lumieres.setStyleSheet(style_Selected)
        else:
            self.bouton_activer_lumieres.setStyleSheet(style_default)

    # def j1_enable_released_callback(self):
    #     if self.j1_enable_state:
    #         self.j1_enable_state = False
    #     else:
    #         self.j1_enable_state = True

    # def ctrl_joy_demux_callback(self):
    #     if '/set_arm_joy' in rosservice.get_service_list():
    #         if self.joy_demux_state:
    #             if(self.set_arm_joy(False)):
    #                 self.joy_demux_state = False;
    #         else:
    #             if(self.set_arm_joy(True)):
    #                 self.joy_demux_state = True;
    #     else:
    #         rospy.logerr("Service 'set_arm_joy' not available")
