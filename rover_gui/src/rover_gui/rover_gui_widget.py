from __future__ import (print_function, absolute_import, division, unicode_literals)

import os
import rospkg
import rospy
from rospkg import RosPack
from python_qt_binding import loadUi
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QObject
from PyQt5.QtGui import QKeySequence
from PyQt5.QtWidgets import QShortcut, QSlider, QLCDNumber
from rover_udes.msg import Command
from sensor_msgs.msg import NavSatFix


class RoverGuiWidget(QtWidgets.QWidget):

    def __init__(self):
        super(RoverGuiWidget, self).__init__()

        ui_file = os.path.join(rospkg.RosPack().get_path('rover_gui'), 'resource', 'rover.ui')
        loadUi(ui_file, self)
        self.setObjectName('RoverGuiWidget')

        self.foward_flag = False
        self.right_flag = False
        self.left_flag = False
        self.backward_flag = False

        self.right = 0.0
        self.left= 0.0
        self.is_active = False

        self.foward_btn.pressed.connect(self.foward_btn_callback)
        self.left_btn.pressed.connect(self.left_btn_callback)
        self.right_btn.pressed.connect(self.right_btn_callback)
        self.backward_btn.pressed.connect(self.backward_btn_callback)

        self.foward_btn.released.connect(self.foward_btn_released_callback)
        self.right_btn.released.connect(self.right_btn_released_callback)
        self.left_btn.released.connect(self.left_btn_released_callback)
        self.backward_btn.released.connect(self.backward_btn_released_callback)

        self.keyboard_check_box.clicked[bool].connect(self.keyboard_check_box_callback)
        
        self.foward_shortcut = QShortcut(QKeySequence(self.tr("W", "File|Open")),self)
        self.right_shortcut = QShortcut(QKeySequence(self.tr("D", "File|Open")),self)
        self.left_shortcut = QShortcut(QKeySequence(self.tr("A", "File|Open")),self)
        self.backward_shortcut = QShortcut(QKeySequence(self.tr("S", "File|Open")),self)

        self.foward_shortcut.activated.connect(self.foward_btn.animateClick)
        self.right_shortcut.activated.connect(self.right_btn.animateClick)
        self.left_shortcut.activated.connect(self.left_btn.animateClick)
        self.backward_shortcut.activated.connect(self.backward_btn.animateClick)

        # Print current position
        self.currentPos_sub = rospy.Subscriber('pos', NavSatFix, self.gps_pos_callback)


        self.gui_cmd_pub = rospy.Publisher('gui_cmd', Command, queue_size=10)
        rospy.Timer(rospy.Duration(1.0/10.0),self.publish_command)
        

    def foward_btn_callback(self):
        self.foward_flag = True
        self.update_command()

    def right_btn_callback(self):
        self.right_flag = True
        self.update_command()

    def left_btn_callback(self):
        self.left_flag = True
        self.update_command()

    def backward_btn_callback(self):
        self.backward_flag = True
        self.update_command()

    def foward_btn_released_callback(self):
        self.foward_flag = False
        self.update_command()

    def right_btn_released_callback(self):
        self.right_flag = False
        self.update_command()

    def left_btn_released_callback(self):
        self.left_flag = False
        self.update_command()

    def backward_btn_released_callback(self):
        self.backward_flag = False
        self.update_command()
    
    def keyboard_check_box_callback(self):
        self.update_command()

    def update_command(self):
        if self.foward_flag:
            self.right = self.speed_slider.value()/100.0
            self.left = self.speed_slider.value()/100.0
        elif self.backward_flag:
            self.right = -(self.speed_slider.value()/100.0)
            self.right = -(self.speed_slider.value()/100.0)
        else:
            self.right = 0.0
            self.left = 0.0

        if self.right_flag:
            self.right = -(self.speed_slider.value()/100.0)
            self.left = self.speed_slider.value()/100.0
        elif self.left_flag:
            self.right = self.speed_slider.value()/100.0
            self.left = -(self.speed_slider.value()/100.0)
        else:
            self.right = 0.0
            self.left = 0.0
            
        self.is_active = self.keyboard_check_box.isChecked()

    def publish_command(self, event):
        command = Command()
        command.linear = self.right
        command.angular = self.left
        command.is_active = self.is_active
        self.gui_cmd_pub.publish(command)

    def gps_pos_callback(self, data):
        self.latDisp.display(data.latitude)
        self.longDisp.display(data.longitude)
