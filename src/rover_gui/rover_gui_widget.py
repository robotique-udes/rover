
import os
import rospkg
import rospy
from rospkg import RosPack
from python_qt_binding import loadUi
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QObject
from PyQt5.QtGui import QKeySequence
from PyQt5.QtWidgets import QShortcut, QSlider
from rover_control.msg import Command


class RoverGuiWidget(QtWidgets.QWidget):

    def __init__(self):
        super(RoverGuiWidget, self).__init__()

        ui_file = os.path.join(rospkg.RosPack().get_path('rover_gui'), 'resource', 'rover.ui')
        loadUi(ui_file, self)
        self.setObjectName('RoverGuiWidget')

        self.left_foward_flag = False
        self.right_foward_flag = False
        self.left_backward_flag = False
        self.right_backward_flag = False

        self.left = 0.0
        self.right = 0.0
        self.is_active = False

        self.left_foward_btn.clicked[bool].connect(self.left_foward_btn_callback)
        self.right_foward_btn.clicked[bool].connect(self.right_foward_btn_callback)
        self.left_backward_btn.clicked[bool].connect(self.left_backward_btn_callback)
        self.right_backward_btn.clicked[bool].connect(self.right_backward_btn_callback)

        self.left_foward_btn.released.connect(self.left_foward_btn_released_callback)
        self.right_foward_btn.released.connect(self.right_foward_btn_released_callback)
        self.left_backward_btn.released.connect(self.left_backward_btn_released_callback)
        self.right_backward_btn.released.connect(self.right_backward_btn_released_callback)
        
        self.left_foward_shortcut = QShortcut(QKeySequence(self.tr("W", "File|Open")),self)
        self.right_foward_shortcut = QShortcut(QKeySequence(self.tr("O", "File|Open")),self)
        self.left_backward_shortcut = QShortcut(QKeySequence(self.tr("S", "File|Open")),self)
        self.right_backward_shortcut = QShortcut(QKeySequence(self.tr("K", "File|Open")),self)

        self.left_foward_shortcut.activated.connect(self.left_foward_btn.animateClick)
        self.right_foward_shortcut.activated.connect(self.right_foward_btn.animateClick)
        self.left_backward_shortcut.activated.connect(self.left_backward_btn.animateClick)
        self.right_backward_shortcut.activated.connect(self.right_backward_btn.animateClick)

        self.gui_cmd_pub = rospy.Publisher('gui_cmd', Command, queue_size=10)
        rospy.Timer(rospy.Duration(1.0/10.0),self.publish_command)
        

    def left_foward_btn_callback(self):
        self.left_foward_flag = True
        self.update_command()

    def right_foward_btn_callback(self):
        self.right_foward_flag = True
        self.update_command()

    def left_backward_btn_callback(self):
        self.left_backward_flag = True
        self.update_command()

    def right_backward_btn_callback(self):
        self.right_backward_flag = True
        self.update_command()

    def left_foward_btn_released_callback(self):
        self.left_foward_flag = False
        self.update_command()

    def right_foward_btn_released_callback(self):
        self.right_foward_flag = False
        self.update_command()

    def left_backward_btn_released_callback(self):
        self.left_backward_flag = False
        self.update_command()

    def right_backward_btn_released_callback(self):
        self.right_backward_flag = False
        self.update_command()

    def update_command(self):
        if self.left_foward_flag:
            self.left = self.speed_slider.value()/100
        elif self.left_backward_flag:
            self.left = -(self.speed_slider.value()/100)
        else:
            self.left = 0

        if self.right_foward_flag:
            self.right = self.speed_slider.value()/100
        elif self.left_backward_flag:
            self.right = -(self.speed_slider.value()/100)
        else:
            self.right = 0
            
        self.is_active = self.keyboard_check_box.isChecked()

    def publish_command(self, event):
        command = Command()
        command.left = self.left
        command.right = self.right
        command.is_active = self.is_active
        self.gui_cmd_pub.publish(command)




