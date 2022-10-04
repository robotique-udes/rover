from __future__ import (print_function, absolute_import, division, unicode_literals)

import os
import rospkg
import rospy
from rospkg import RosPack
from python_qt_binding import loadUi
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QObject
from PyQt5.QtGui import QKeySequence
from PyQt5.QtWidgets import QShortcut, QSlider, QAbstractSlider, QSpinBox
from rover_udes.msg import CamCommand
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


class RoverCamControlWidget(QtWidgets.QWidget):

    def __init__(self):
        super(RoverCamControlWidget, self).__init__()

        ui_file = os.path.join(rospkg.RosPack().get_path('rover_cam_control'), 'resource', 'cam_control.ui')
        loadUi(ui_file, self)
        self.setObjectName('RoverCamControlWidget')

        self.is_active = False

        self.cam_horizontal_pos = 0
        self.cam_vertical_pos = 0

        self.cam_horizontal_pos_signal = 0
        self.cam_vertical_pos_signal = 0

        self.init_fields()

        self.cam_left_btn.clicked.connect(self.cam_left_btn_callback)
        self.cam_right_btn.clicked.connect(self.cam_right_btn_callback)
        self.cam_up_btn.clicked.connect(self.cam_up_btn_callback)
        self.cam_down_btn.clicked.connect(self.cam_down_btn_callback)
        self.cam_update_btn.clicked.connect(self.update_command)
        self.cam_reset_btn.clicked.connect(self.init_fields)

        self.cam_keyboard_check_box.clicked[bool].connect(self.cam_keyboard_check_box_callback)
        
        self.cam_left_shortcut = QShortcut(QKeySequence(self.tr("G", "File|Open")), self)
        self.cam_right_shortcut = QShortcut(QKeySequence(self.tr("J", "File|Open")), self)
        self.cam_up_shortcut = QShortcut(QKeySequence(self.tr("Y", "File|Open")), self)
        self.cam_down_shortcut = QShortcut(QKeySequence(self.tr("H", "File|Open")), self)
        self.cam_update_shortcut = QShortcut(QKeySequence(self.tr("B", "File|Open")), self)

        self.cam_left_shortcut.activated.connect(self.cam_left_btn.animateClick)
        self.cam_right_shortcut.activated.connect(self.cam_right_btn.animateClick)
        self.cam_up_shortcut.activated.connect(self.cam_up_btn.animateClick)
        self.cam_down_shortcut.activated.connect(self.cam_down_btn.animateClick)
        self.cam_update_shortcut.activated.connect(self.cam_update_btn.animateClick)

        self.cam_horizontal_slider.valueChanged.connect(self.update_fields)
        self.cam_vertical_slider.valueChanged.connect(self.update_fields)
        self.cam_horizontal_field.valueChanged.connect(self.update_sliders)
        self.cam_vertical_field.valueChanged.connect(self.update_sliders)

        self.cam_cmd_pub = rospy.Publisher('gui_cmd_ptu', CamCommand, queue_size=10)
        rospy.Timer(rospy.Duration(1.0/10.0), self.publish_command)

        self.photo_cmd_pub = rospy.Publisher('take_photo', Bool, queue_size=10)
        self.pano_cmd_pub = rospy.Publisher('stitch_pano', Bool, queue_size=10)

        self.pano_btn.clicked.connect(self.change_pano_state)
        self.take_photo_btn.clicked.connect(self.publish_photo_cmd)
        self.stitch_pano_btn.clicked.connect(self.publish_pano_cmd)

    def init_fields(self):
        self.cam_horizontal_field.setValue(0)
        self.cam_horizontal_slider.setValue(0)
        self.cam_horizontal_pos_info.setValue(0)
        self.cam_vertical_field.setValue(0)
        self.cam_vertical_slider.setValue(0)
        self.cam_vertical_pos_info.setValue(0)
        self.cam_keyboard_check_box.setChecked(False)

        self.cam_horizontal_pos = 0
        self.cam_vertical_pos = 0

        self.cam_horizontal_pos_signal = 0
        self.cam_vertical_pos_signal = 0

        self.take_photo_btn.setEnabled(False)
        self.stitch_pano_btn.setEnabled(False)

        self.update_command()

    def cam_left_btn_callback(self):
        self.cam_horizontal_slider.triggerAction(2)
        self.update_fields()
        self.update_command()

    def cam_right_btn_callback(self):
        self.cam_horizontal_slider.triggerAction(1)
        self.update_fields()
        self.update_command()

    def cam_up_btn_callback(self):
        self.cam_vertical_slider.triggerAction(1)
        self.update_fields()
        self.update_command()

    def cam_down_btn_callback(self):
        self.cam_vertical_slider.triggerAction(2)
        self.update_fields()
        self.update_command()

    def cam_keyboard_check_box_callback(self):
        self.update_command()

    def update_sliders(self):
        self.cam_horizontal_pos = self.cam_horizontal_field.value()
        self.cam_vertical_pos = self.cam_vertical_field.value()
        self.cam_horizontal_slider.setValue(self.cam_horizontal_pos)
        self.cam_vertical_slider.setValue(self.cam_vertical_pos)

        self.cam_update_btn.setDown(False)

    def update_fields(self):
        self.cam_horizontal_pos = self.cam_horizontal_slider.value()
        self.cam_vertical_pos = self.cam_vertical_slider.value()
        self.cam_horizontal_field.setValue(self.cam_horizontal_pos)
        self.cam_vertical_field.setValue(self.cam_vertical_pos)

        self.cam_update_btn.setDown(False)

    def update_command(self):
        self.cam_horizontal_pos_signal = self.cam_horizontal_pos
        self.cam_vertical_pos_signal = self.cam_vertical_pos
        self.cam_horizontal_pos_info.setValue(self.cam_horizontal_pos_signal)
        self.cam_vertical_pos_info.setValue(self.cam_vertical_pos_signal)

        self.cam_update_btn.setDown(True)

        self.is_active = self.cam_keyboard_check_box.isChecked()

    def publish_command(self, event):
        command = CamCommand()
        command.mode = 0  # position mode
        command.cam_horizontal = self.cam_horizontal_pos_signal
        command.cam_vertical = self.cam_vertical_pos_signal
        #command.is_active = self.is_active
        self.cam_cmd_pub.publish(command)

    def publish_photo_cmd(self, event):
        msg = Bool()
        msg.data = True
        self.photo_cmd_pub.publish(msg)

    def publish_pano_cmd(self, event):
        msg = Bool()
        msg.data = True
        self.pano_cmd_pub.publish(msg)

    def change_pano_state(self, event):
        if self.pano_btn.text() == 'Panorama ON':
            turn_on = True
        else:
            turn_on = False
        
        try:
            rospy.wait_for_service('change_pano_state', 0.5)
            change_pano_state = rospy.ServiceProxy('change_pano_state', SetBool)
            request = SetBool()
            request.data = turn_on
            response = change_pano_state(turn_on)
            if response.success:
                self.take_photo_btn.setEnabled(True)
                self.stitch_pano_btn.setEnabled(True)
                self.pano_btn.setText('Panorama OFF')
            else:
                self.take_photo_btn.setEnabled(False)
                self.stitch_pano_btn.setEnabled(False)
                self.pano_btn.setText('Panorama ON')
        except:
            print("Service call failed")
