from __future__ import (print_function, absolute_import, division, unicode_literals)

import os
import rospkg
import rospy
import roslaunch
import os
from python_qt_binding import loadUi
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QLabel, QPushButton, QAbstractButton
from threading import Lock

#Styling "MACROS"
STYLE_DEFAULT = ""
STYLE_SELECTED = "color: white; background-color: green"
STYLE_DISABLE = "color: white; background-color: grey"
STYLE_LIMITING = "color: black; background-color: yellow"
STYLE_WARN = "color: black; background-color: rgb(255, 124, 0);"

class LaunchInterface():
    def __init__(self, uuid, pkg_name: str, launchfile_name: str):
        self.name_pkg = pkg_name
        self.name_launchfile = launchfile_name
        self.uuid = uuid
        self.launchfile = roslaunch.rlutil.resolve_launch_arguments([pkg_name, launchfile_name + '.launch'])
        self.launch_handler = roslaunch.parent.ROSLaunchParent(uuid, self.launchfile)
        self.launch_handler_started: bool = False

class RoverLaunchControlWidget(QtWidgets.QWidget):
    
    def __init__(self):
        super(RoverLaunchControlWidget, self).__init__()

        ui_file = os.path.join(rospkg.RosPack().get_path('rover_launch_control'), 'resource', 'rover_launch_control.ui')
        loadUi(ui_file, self)
        self.setObjectName('RoverLaunchControlWidget')

        self.lock = Lock()

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)


        #Button callbacks
        joints_interface = LaunchInterface(uuid= uuid, pkg_name= 'rover_arm', launchfile_name='joints')
        self.pb_joints.released.connect(lambda: self.launchFile(joints_interface, self.pb_joints))

    def launchFile(self, launch_interface: LaunchInterface, button: QPushButton):
        with self.lock:
            if not launch_interface.launch_handler_started:
                launch_interface.launch_handler = roslaunch.parent.ROSLaunchParent(launch_interface.uuid, launch_interface.launchfile)
                launch_interface.launch_handler_started = True
                button.setStyleSheet(STYLE_SELECTED)
                launch_interface.launch_handler.start()
            else:
                launch_interface.launch_handler_started = False
                button.setStyleSheet(STYLE_DEFAULT)
                launch_interface.launch_handler.shutdown()
                rospy.logwarn(launch_interface.name_pkg + ' ' + launch_interface.name_launchfile + '.launch shutdowned')
