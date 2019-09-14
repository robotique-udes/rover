
import os
import rospkg
import rospy
from rospkg import RosPack
from python_qt_binding import loadUi
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QObject
from PyQt5.QtGui import QKeySequence
from PyQt5.QtWidgets import QShortcut



class RoverGuiWidget(QtWidgets.QWidget):

    def __init__(self):
        super(RoverGuiWidget, self).__init__()

        ui_file = os.path.join(rospkg.RosPack().get_path('rover_gui'), 'resource', 'rover.ui')
        loadUi(ui_file, self)
        self.setObjectName('RoverGuiWidget')

        self.left_foward_btn.clicked[bool].connect(self.left_foward_btn_callback)
        self.right_foward_btn.clicked[bool].connect(self.right_foward_btn_callback)
        self.left_backward_btn.clicked[bool].connect(self.left_backward_btn_callback)
        self.right_backward_btn.clicked[bool].connect(self.right_backward_btn_callback)
        
        self.left_foward_shortcut = QShortcut(QKeySequence(self.tr("W", "File|Open")),self)
        self.right_foward_shortcut = QShortcut(QKeySequence(self.tr("O", "File|Open")),self)
        self.left_backward_shortcut = QShortcut(QKeySequence(self.tr("S", "File|Open")),self)
        self.right_backward_shortcut = QShortcut(QKeySequence(self.tr("K", "File|Open")),self)

        self.left_foward_shortcut.activated.connect(self.left_foward_btn.animateClick)
        self.right_foward_shortcut.activated.connect(self.right_foward_btn.animateClick)
        self.left_backward_shortcut.activated.connect(self.left_backward_btn.animateClick)
        self.right_backward_shortcut.activated.connect(self.right_backward_btn.animateClick)
        

    def left_foward_btn_callback(self):
        if self.keyboard_check_box.isChecked() and not self.controller_check_box.isChecked():
            print("Left_foward")
        else:
            print("Keyboard disabled")

    def right_foward_btn_callback(self):
        if self.keyboard_check_box.isChecked() and not self.controller_check_box.isChecked():
            print("Right_foward")
        else:
            print("Keyboard disabled")

    def left_backward_btn_callback(self):
        if self.keyboard_check_box.isChecked() and not self.controller_check_box.isChecked():
            print("Left_backward")
        else:
            print("Keyboard disabled")

    def right_backward_btn_callback(self):
        if self.keyboard_check_box.isChecked() and not self.controller_check_box.isChecked():
            print("Right_backward")
        else:
            print("Keyboard disabled")



