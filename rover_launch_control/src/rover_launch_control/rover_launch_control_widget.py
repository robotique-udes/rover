from __future__ import (print_function, absolute_import, division, unicode_literals)

import os
import rospkg
import rospy
import roslaunch
import rosparam
import os
from python_qt_binding import loadUi
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QLabel, QPushButton, QAbstractButton, QComboBox
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
        self.launch_handler_started: bool = False
        self.launch_handler = None

class RoverLaunchControlWidget(QtWidgets.QWidget):
    
    def __init__(self):
        super(RoverLaunchControlWidget, self).__init__()

        ui_file = os.path.join(rospkg.RosPack().get_path('rover_launch_control'), 'resource', 'rover_launch_control.ui')
        loadUi(ui_file, self)
        self.setObjectName('RoverLaunchControlWidget')

        # I don't really understands qt and ros python threading so putting some locks on all button callback to only 
        # execute one at a time (probably useless but more secure nonetheless)
        self.lockLaunchFile = Lock()
        self.lockLaunchCameraStream = Lock()

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        self.resolutionDict = {
            "1080p": ("1920", "1080"),
            "720p": ("1080", "720"),
            "480p": ("640", "480"),
            "360p": ("480", "360"),
            "144p": ("192", "144"),
            }

        #rover_control
        base_interface = LaunchInterface(uuid= uuid, pkg_name= 'rover_control', launchfile_name='base')
        self.pb_base.released.connect(lambda: self.launchFile(base_interface, self.pb_base))

        rover_interface = LaunchInterface(uuid= uuid, pkg_name= 'rover_control', launchfile_name='rover')
        self.pb_rover.released.connect(lambda: self.launchFile(rover_interface, self.pb_rover))

        #rover_arm
        joints_interface = LaunchInterface(uuid= uuid, pkg_name= 'rover_arm', launchfile_name='joints')
        self.pb_joints.released.connect(lambda: self.launchFile(joints_interface, self.pb_joints))

        controller_interface = LaunchInterface(uuid= uuid, pkg_name= 'rover_arm', launchfile_name='controller')
        self.pb_controller.released.connect(lambda: self.launchFile(controller_interface, self.pb_controller))

        #cameras
        cam_arducam_interface = LaunchInterface(uuid= uuid, pkg_name= 'rover_control', launchfile_name='cam_arducam')
        self.pb_cam_arducam_launch.released.connect(lambda: self.launchCameraStream("cam_arducam",
                                                                                     120,
                                                                                     cam_arducam_interface,
                                                                                     self.pb_cam_arducam_launch,
                                                                                     self.cb_cam_arducam_resolution,
                                                                                     self.pb_current_cam_arducam_resolution,
                                                                                     self.cb_cam_arducam_framerate,
                                                                                     self.pb_current_cam_arducam_framerate))
        self.pb_cam_arducam_apply.released.connect(lambda: self.ApplyCameraStream("cam_arducam",
                                                                                   120,
                                                                                   cam_arducam_interface,
                                                                                   self.pb_cam_arducam_launch,
                                                                                   self.cb_cam_arducam_resolution,
                                                                                   self.pb_current_cam_arducam_resolution,
                                                                                   self.cb_cam_arducam_framerate,
                                                                                   self.pb_current_cam_arducam_framerate))

        cam_sonix_interface = LaunchInterface(uuid= uuid, pkg_name= 'rover_control', launchfile_name='cam_sonix')
        self.pb_cam_sonix_launch.released.connect(lambda: self.launchCameraStream("cam_sonix",
                                                                                     120,
                                                                                     cam_sonix_interface,
                                                                                     self.pb_cam_sonix_launch,
                                                                                     self.cb_cam_sonix_resolution,
                                                                                     self.pb_current_cam_sonix_resolution,
                                                                                     self.cb_cam_sonix_framerate,
                                                                                     self.pb_current_cam_sonix_framerate))
        self.pb_cam_sonix_apply.released.connect(lambda: self.ApplyCameraStream("cam_sonix",
                                                                                   120,
                                                                                   cam_sonix_interface,
                                                                                   self.pb_cam_sonix_launch,
                                                                                   self.cb_cam_sonix_resolution,
                                                                                   self.pb_current_cam_sonix_resolution,
                                                                                   self.cb_cam_sonix_framerate,
                                                                                   self.pb_current_cam_sonix_framerate))

    def launchFile(self, launch_interface: LaunchInterface, button: QPushButton):
        with self.lockLaunchFile:
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

    def launchCameraStream(self,
                           cameraName: str,
                           cameraFramerate:int,
                           launch_interface:LaunchInterface,
                           button: QPushButton,
                           cb_resolution: QComboBox,
                           pb_current_resolution: QPushButton,
                           cb_framerate:QComboBox,
                           pb_current_framerate: QPushButton):
        with self.lockLaunchCameraStream:
            if not launch_interface.launch_handler_started:
                rosparam.set_param(cameraName + '/hardware/framerate', str(cameraFramerate))
                framerate = int(cb_framerate.currentText())
                param_skip = int(cameraFramerate/framerate)-1 
                rosparam.set_param(cameraName + '/compressed_packet_controller/skip', str(param_skip))
                
                if ((param_skip + 1) * framerate != 120):
                    rospy.logerr("Wrong framerate parameters node can't start")
                    return

                try:
                    resolution = cb_resolution.currentText()
                except KeyError:
                    rospy.logerr("Selected resolution isn't defined ")
                    return
                
                param_imageWidth = self.resolutionDict[resolution][0]
                param_imageHeight = self.resolutionDict[resolution][1]
                rosparam.set_param(cameraName + '/hardware/image_width', param_imageWidth)
                rosparam.set_param(cameraName + '/image_height', param_imageHeight)
                
                rospy.logwarn("Selected resolution: " + resolution + "(" + param_imageWidth + "x" + param_imageHeight + ")")
                rospy.logwarn("selected framerate: " + str(framerate))

                pb_current_framerate.setText(str(framerate))
                pb_current_resolution.setText(resolution)
            else:
                pb_current_framerate.setText("")
                pb_current_resolution.setText("")


            self.launchFile(launch_interface, button)

    def ApplyCameraStream(self,
                cameraName: str,
                cameraFramerate:int,
                launch_interface:LaunchInterface,
                button: QPushButton,
                cb_resolution: QComboBox,
                pb_current_resolution: QPushButton,
                cb_framerate:QComboBox,
                pb_current_framerate: QPushButton):
    
        if launch_interface.launch_handler_started:
            for i in range(2):
                self.launchCameraStream(cameraName, cameraFramerate, launch_interface, button, cb_resolution, pb_current_resolution, cb_framerate, pb_current_framerate)
        else:
            rospy.logwarn(cameraName + " is not started yet")    
