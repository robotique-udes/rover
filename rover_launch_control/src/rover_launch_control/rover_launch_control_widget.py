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
from PyQt5.QtWidgets import QShortcut, QSlider, QLCDNumber, QLabel, QPushButton, QFrame, QListView
import dynamic_reconfigure.client

#Styling "MACROS"
style_default = ""
style_Selected = "color: white; background-color: green"
style_Disable = "color: white; background-color: grey"
style_Limiting = "color: black; background-color: yellow"
style_Warn = "color: black; background-color: rgb(255, 124, 0);"

#DEFINE
CLIENT_TIMEOUT: int = 30

class RoverLaunchControlWidget(QtWidgets.QWidget):
    
    def __init__(self):
        super(RoverLaunchControlWidget, self).__init__()

        ui_file = os.path.join(rospkg.RosPack().get_path('rover_launch_control'), 'resource', 'rover_launch_control.ui')
        loadUi(ui_file, self)
        self.setObjectName('RoverLaunchControlWidget')
        # self.is_active = False

        self.cam_sonix_client = dynamic_reconfigure.client.Client("/cam_sonix/image/theora")

        self.fps_slider.sliderReleased.connect(self.updateValueFps)

    def updateValueFps(self):
        size: int = self.fps_slider.value()
        print(self.cam_sonix_client.get_configuration(timeout=CLIENT_TIMEOUT))
