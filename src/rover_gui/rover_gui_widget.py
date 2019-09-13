
import os
import rospkg
import rospy
from rospkg import RosPack
from python_qt_binding import loadUi
from PyQt5 import QtCore, QtGui, QtWidgets


class RoverGuiWidget(QtWidgets.QWidget):

    def __init__(self):
        super(RoverGuiWidget, self).__init__()

        ui_file = os.path.join(rospkg.RosPack().get_path('rover_gui'), 'resource', 'rover.ui')
        loadUi(ui_file, self)
        self.setObjectName('RoverGuiWidget')