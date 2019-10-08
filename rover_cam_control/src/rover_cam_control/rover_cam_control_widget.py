
import os
import rospkg
import rospy
from rospkg import RosPack
from python_qt_binding import loadUi
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QObject
from PyQt5.QtGui import QKeySequence
from PyQt5.QtWidgets import QShortcut, QSlider, QAbstractSlider, QSpinBox
# from rover_control.msg import Command


class RoverCamControlWidget(QtWidgets.QWidget):

    def __init__(self):
        super(RoverCamControlWidget, self).__init__()

        ui_file = os.path.join(rospkg.RosPack().get_path('rover_cam_control'), 'resource', 'cam_control.ui')
        loadUi(ui_file, self)
        self.setObjectName('RoverCamControlWidget')

        self.up_cam_flag = False
        self.down_cam_flag = False
        self.left_cam_flag = False
        self.right_cam_flag = False

        self.cam_horizontal_slider
        self.cam_vertical_slider

        self.cam_horizontal_field.setValue(self.cam_horizontal_slider.value())
        self.cam_vertical_field.setValue(self.cam_vertical_slider.value())

        self.left = 0.0
        self.right = 0.0
        self.is_active = False

        self.cam_left_btn.clicked.connect(self.cam_left_btn_callback)
        self.cam_right_btn.clicked.connect(self.cam_right_btn_callback)
        self.cam_up_btn.clicked.connect(self.cam_up_btn_callback)
        self.cam_down_btn.clicked.connect(self.cam_down_btn_callback)

        # self.cam_left_btn.released.connect(self.cam_left_btn_released_callback)
        # self.cam_right_btn.released.connect(self.cam_right_btn_released_callback)
        # self.cam_up_btn.released.connect(self.cam_up_btn_released_callback)
        # self.cam_down_btn.released.connect(self.cam_down_btn_released_callback)

        self.keyboard_check_box.clicked[bool].connect(self.keyboard_check_box_callback)
        
        self.cam_left_shortcut = QShortcut(QKeySequence(self.tr("G", "File|Open")), self)
        self.cam_right_shortcut = QShortcut(QKeySequence(self.tr("J", "File|Open")), self)
        self.cam_up_shortcut = QShortcut(QKeySequence(self.tr("Y", "File|Open")), self)
        self.cam_down_shortcut = QShortcut(QKeySequence(self.tr("H", "File|Open")), self)

        self.cam_left_shortcut.activated.connect(self.cam_left_btn.animateClick)
        self.cam_right_shortcut.activated.connect(self.cam_right_btn.animateClick)
        self.cam_up_shortcut.activated.connect(self.cam_up_btn.animateClick)
        self.cam_down_shortcut.activated.connect(self.cam_down_btn.animateClick)

        # self.gui_cmd_pub = rospy.Publisher('gui_cmd', Command, queue_size=10)
        # rospy.Timer(rospy.Duration(1.0/10.0), self.publish_command)
        
    def cam_left_btn_callback(self):
        self.cam_horizontal_slider.triggerAction(2)
        # self.update_command()
        self.update_fields()

    def cam_right_btn_callback(self):
        self.cam_horizontal_slider.triggerAction(1)
        # self.update_command()
        self.update_fields()

    def cam_up_btn_callback(self):
        self.cam_vertical_slider.triggerAction(1)
        # self.update_command()
        self.update_fields()

    def cam_down_btn_callback(self):
        self.cam_vertical_slider.triggerAction(2)
        # self.update_command()
        self.update_fields()

    def cam_left_btn_released_callback(self):
        self.left_cam_flag = False
        self.update_command()

    def cam_right_btn_released_callback(self):
        self.right_cam_flag = False
        self.update_command()

    def cam_up_btn_released_callback(self):
        self.up_cam_flag = False
        self.update_command()

    def cam_down_btn_released_callback(self):
        self.down_cam_flag = False
        self.update_command()
    
    def keyboard_check_box_callback(self):
        self.update_command()

    def update_fields(self):
        self.cam_horizontal_field.setValue(self.cam_horizontal_slider.value())
        self.cam_vertical_field.setValue(self.cam_vertical_slider.value())

    # def update_command(self):
    #     if self.left_foward_flag:
    #         self.left = self.speed_slider.value()/100.0
    #     elif self.left_backward_flag:
    #         self.left = -(self.speed_slider.value()/100.0)
    #     else:
    #         self.left = 0
    #
    #     if self.right_foward_flag:
    #         self.right = self.speed_slider.value()/100.0
    #     elif self.right_backward_flag:
    #         self.right = -(self.speed_slider.value()/100.0)
    #     else:
    #         self.right = 0
            
        self.is_active = self.keyboard_check_box.isChecked()

    # def publish_command(self, event):
    #     command = Command()
    #     command.left = self.left
    #     command.right = self.right
    #     command.is_active = self.is_active
    #     self.gui_cmd_pub.publish(command)




