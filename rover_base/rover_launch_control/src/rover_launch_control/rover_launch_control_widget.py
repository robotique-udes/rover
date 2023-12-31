from __future__ import (print_function, absolute_import, division, unicode_literals)

import os
import rospkg
import rospy
import roslaunch
import rosparam
import rostopic
import os
import time
from python_qt_binding import loadUi
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtWidgets import QLabel, QPushButton, QAbstractButton, QComboBox, QApplication
from threading import Lock
from copy import copy
import rosservice

# Styling "MACROS"
STYLE_DEFAULT = ""
STYLE_SELECTED = "color: white; background-color: green"
STYLE_DISABLE = "color: white; background-color: grey"
STYLE_LIMITING = "color: black; background-color: yellow"
STYLE_WARN = "color: black; background-color: rgb(255, 124, 0);"

# Global Variables
launchmode_local = 'false'

class LaunchInterface():
    def __init__(self, uuid, pkg_name: str, launchfile_name: str, parameters: list = []):
        self.name_pkg = pkg_name
        self.name_launchfile = launchfile_name
        self.parameters = parameters
        self.uuid = uuid
        self.launch_handler_started: bool = False
        self.launch_handler = None

    def update_param(self):
        self.launchfile = roslaunch.rlutil.resolve_launch_arguments([self.name_pkg, self.name_launchfile + '.launch'])
        if self.parameters == None:
            parameters = ['local_only:=' + launchmode_local]
        else:
            parameters = copy(self.parameters)
            parameters = ['local_only:=' + launchmode_local]

        self.launchfile = [(self.launchfile[0], parameters)]

class BandwidthInterface:
    def __init__(self, topic_name, window: float = 5.0):
        self.topic_name = topic_name
        self.window = window
        self.bw_api = rostopic.ROSTopicBandwidth(self.window);
        self.sub = rospy.Subscriber(self.topic_name, rospy.AnyMsg, self.bw_api.callback)
        self._flag: bool = False
        self.bw = None
        self.t0 = None
        self.tn = None

    def get_bw(self) -> int :
        """print the average publishing rate to screen"""
        if len(self.bw_api.times) < 2:
            return
        with self.bw_api.lock:
            n = len(self.bw_api.times)
            tn = time.time()
            t0 = self.bw_api.times[0]
            
            total = sum(self.bw_api.sizes)
            bytes_per_s = total / (tn - t0)
            mean = total / n

            # min and max
            max_s = max(self.bw_api.sizes)
            min_s = min(self.bw_api.sizes)

        #min/max and even mean are likely to be much smaller, but for now I prefer unit consistency
        if bytes_per_s < 1000:
            bw, mean, min_s, max_s = ["%.2fB"%v for v in [bytes_per_s, mean, min_s, max_s]]
        elif bytes_per_s < 1000000:
            bw, mean, min_s, max_s = ["%.2fKB"%(v/1000) for v in [bytes_per_s, mean, min_s, max_s]]  
        else:
            bw, mean, min_s, max_s = ["%.2fMB"%(v/1000000) for v in [bytes_per_s, mean, min_s, max_s]]
        
        self.bw = bw
        self.t0 = t0
        self.tn = tn
        return

class RoverLaunchControlWidget(QtWidgets.QWidget):
    
    def __init__(self):
        self.name = "RoverLaunchControlWidget"
        super(RoverLaunchControlWidget, self).__init__()

        ui_file = os.path.join(rospkg.RosPack().get_path('rover_launch_control'), 'resource', 'rover_launch_control.ui')
        loadUi(ui_file, self)
        self.setObjectName(self.name)

        # I don't really understands qt and ros python threading so putting some locks on all button callback to only 
        # execute one at a time (probably useless but more secure nonetheless)
        self.lockLaunchFile = Lock()
        

        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        uuid = self.uuid
        roslaunch.configure_logging(uuid)

        #Mode selection buttons
        #setting default value
        global launchmode_local
        launchmode_local = 'true' 

        if launchmode_local == 'true':
            self.pb_local_true.setStyleSheet(STYLE_SELECTED)
            self.pb_local_false.setStyleSheet(STYLE_WARN)
        elif launchmode_local == 'false':
            self.pb_local_false.setStyleSheet(STYLE_SELECTED)
            self.pb_local_false.setStyleSheet(STYLE_WARN)
        else:
            exit()
            
        self.pb_local_true.released.connect(lambda: self.localModeSelection(True, self.pb_local_true, [self.pb_local_false]))
        self.pb_local_false.released.connect(lambda: self.localModeSelection(False, self.pb_local_false, [self.pb_local_true]))

        #rover_control
        base_interface = LaunchInterface(uuid= uuid, pkg_name= 'rover_control', launchfile_name='base')
        self.pb_base.released.connect(lambda: self.launchFile(base_interface, self.pb_base))

        rover_interface = LaunchInterface(uuid= uuid, pkg_name= 'rover_control', launchfile_name='rover')
        self.pb_rover.released.connect(lambda: self.launchFile(rover_interface, self.pb_rover))

        can_interface = LaunchInterface(uuid= uuid, pkg_name= 'rover_control', launchfile_name='can')
        self.pb_can.released.connect(lambda: self.launchFile(can_interface, self.pb_can))

        camera_control_server_interface = LaunchInterface(uuid= uuid, pkg_name= 'rover_control', launchfile_name='camera_control_server')
        self.pb_panorama_server.released.connect(lambda: self.launchFile(camera_control_server_interface, self.pb_panorama_server))

        #rover_arm
        joints_interface = LaunchInterface(uuid= uuid, pkg_name= 'rover_arm', launchfile_name='joints')
        self.pb_joints.released.connect(lambda: self.launchFile(joints_interface, self.pb_joints))

        controller_interface = LaunchInterface(uuid= uuid, pkg_name= 'rover_arm', launchfile_name='controller')
        self.pb_controller.released.connect(lambda: self.launchFile(controller_interface, self.pb_controller))

        #UI cameras
        self.pb_launch_cam.released.connect(lambda: self.openPopup(CameraLaunch))

        
        

    def localModeSelection(self, value: bool, pb_toggle_self: QPushButton, pb_toggle_friends: "list[str]"):
        global launchmode_local
        launchmode_local = str(value)
        pb_toggle_self.setStyleSheet(STYLE_SELECTED)
        for friends in pb_toggle_friends:
            friends.setStyleSheet(STYLE_DEFAULT)
        rospy.loginfo(rospy.get_name() + "(" + self.name + "): " + "Already launched nodes needs to be restarted for mode to apply")

    def launchFile(self, launch_interface: LaunchInterface, button: QPushButton):
        with self.lockLaunchFile:
            QApplication.setOverrideCursor(QtCore.Qt.WaitCursor)
            try:
                if not launch_interface.launch_handler_started:
                    launch_interface.update_param()
                    launch_interface.launch_handler = roslaunch.parent.ROSLaunchParent(launch_interface.uuid, launch_interface.launchfile)
                    launch_interface.launch_handler_started = True
                    button.setStyleSheet(STYLE_SELECTED)
                    rospy.loginfo(rospy.get_name() + "(" + self.name + "): " + "Starting " + launch_interface.launchfile[0][0] + ' ' + ' '.join(launch_interface.launchfile[0][1]))
                    launch_interface.launch_handler.start()
                else:
                    launch_interface.launch_handler_started = False
                    button.setStyleSheet(STYLE_DEFAULT)
                    launch_interface.launch_handler.shutdown()

                    rospy.logwarn(rospy.get_name() + "(" + self.name + "): " + launch_interface.name_pkg + ' ' + launch_interface.name_launchfile + '.launch shutdowned')
            except:
                button.setStyleSheet(STYLE_WARN);
            
            finally:
                QApplication.restoreOverrideCursor()

    

    # Helper: Open a popup of another QWidget class
    def openPopup(self, class_type):
        self.popUp = class_type(self) 
        self.popUp.show()

# Qwidget for lauching camera
class CameraLaunch(QtWidgets.QWidget):
    def __init__(self, mainWindowsClass: RoverLaunchControlWidget):
        super().__init__()
        ui_file = os.path.join(rospkg.RosPack().get_path('rover_launch_control'), 'resource', 'rover_launch_camera.ui')
        loadUi(ui_file, self)
        self.setObjectName('Launch Camera')

        self.mainWindowsClass = mainWindowsClass
        uuid = self.mainWindowsClass.uuid

        self.name = "RoverLaunchControlWidget"

        self.lockLaunchCameraStream = Lock()

        # self.pb_add_new_waypoint: QPushButton
        # self.pb_add_new_waypoint.released.connect(lambda: self.addWaypoint(self.pb_add_new_waypoint))
        # self.pb_add_new_waypoint_shifted.released.connect(lambda: self.addShiftedWaypoint(self.pb_add_new_waypoint_shifted))
        # self.cb_shifted_waypoint_type_selector: QComboBox

        self.resolutionDict = {
            "1080p": ("1920", "1080"),
            "720p": ("1080", "720"),
            "480p": ("640", "480"),
            "360p": ("480", "360"),
            "144p": ("192", "144"),
            }
        
        #Periodic tasks
        cam_arducam_bandwidth = BandwidthInterface("/cam_arducam/packet/compressed")
        self.bandwidth_updater = rospy.Timer(rospy.Duration(0.5), lambda x: self.bandwidthInfoUpdate(self.bandwidth_updater, cam_arducam_bandwidth, self.pb_current_cam_arducam_bw));
    
        cam_main_bandwidth = BandwidthInterface("/cam_main/packet/compressed")
        self.bandwidth_updater = rospy.Timer(rospy.Duration(0.5), lambda x: self.bandwidthInfoUpdate(self.bandwidth_updater, cam_main_bandwidth, self.pb_current_cam_main_bw));
    
        cam_arriere_bandwidth = BandwidthInterface("/cam_arriere/packet/compressed")
        self.bandwidth_updater = rospy.Timer(rospy.Duration(0.5), lambda x: self.bandwidthInfoUpdate(self.bandwidth_updater, cam_arriere_bandwidth, self.pb_current_cam_arriere_bw));
    
        cam_desaxee_bandwidth = BandwidthInterface("/cam_desaxee/packet/compressed")
        self.bandwidth_updater = rospy.Timer(rospy.Duration(0.5), lambda x: self.bandwidthInfoUpdate(self.bandwidth_updater, cam_desaxee_bandwidth, self.pb_current_cam_desaxee_bw));

        cam_bras_bandwidth = BandwidthInterface("/cam_bras/packet/compressed")
        self.bandwidth_updater = rospy.Timer(rospy.Duration(0.5), lambda x: self.bandwidthInfoUpdate(self.bandwidth_updater, cam_bras_bandwidth, self.pb_current_cam_bras_bw));
    
        cam_pelle_bandwidth = BandwidthInterface("/cam_pelle/packet/compressed")
        self.bandwidth_updater = rospy.Timer(rospy.Duration(0.5), lambda x: self.bandwidthInfoUpdate(self.bandwidth_updater, cam_pelle_bandwidth, self.pb_current_cam_pelle_bw));
    
        cam_base_bras_bandwidth = BandwidthInterface("/cam_base_bras/packet/compressed")
        self.bandwidth_updater = rospy.Timer(rospy.Duration(0.5), lambda x: self.bandwidthInfoUpdate(self.bandwidth_updater, cam_base_bras_bandwidth, self.pb_current_cam_base_bras_bw));
    

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

        cam_main_interface = LaunchInterface(uuid= uuid, pkg_name= 'rover_control', launchfile_name='cam_main')
        self.pb_cam_main_launch.released.connect(lambda: self.launchCameraStream("cam_main",
                                                                                     30,
                                                                                     cam_main_interface,
                                                                                     self.pb_cam_main_launch,
                                                                                     self.cb_cam_main_resolution,
                                                                                     self.pb_current_cam_main_resolution,
                                                                                     self.cb_cam_main_framerate,
                                                                                     self.pb_current_cam_main_framerate))
        self.pb_cam_main_apply.released.connect(lambda: self.ApplyCameraStream("cam_main",
                                                                                   30,
                                                                                   cam_main_interface,
                                                                                   self.pb_cam_main_launch,
                                                                                   self.cb_cam_main_resolution,
                                                                                   self.pb_current_cam_main_resolution,
                                                                                   self.cb_cam_main_framerate,
                                                                                   self.pb_current_cam_main_framerate))
        
        cam_arriere_interface = LaunchInterface(uuid= uuid, pkg_name= 'rover_control', launchfile_name='cam_arriere')
        self.pb_cam_arriere_launch.released.connect(lambda: self.launchCameraStream("cam_arriere",
                                                                                     30,
                                                                                     cam_arriere_interface,
                                                                                     self.pb_cam_arriere_launch,
                                                                                     self.cb_cam_arriere_resolution,
                                                                                     self.pb_current_cam_arriere_resolution,
                                                                                     self.cb_cam_arriere_framerate,
                                                                                     self.pb_current_cam_arriere_framerate))
        self.pb_cam_arriere_apply.released.connect(lambda: self.ApplyCameraStream("cam_arriere",
                                                                                   30,
                                                                                   cam_arriere_interface,
                                                                                   self.pb_cam_arriere_launch,
                                                                                   self.cb_cam_arriere_resolution,
                                                                                   self.pb_current_cam_arriere_resolution,
                                                                                   self.cb_cam_arriere_framerate,
                                                                                   self.pb_current_cam_arriere_framerate))
        
        cam_desaxee_interface = LaunchInterface(uuid= uuid, pkg_name= 'rover_control', launchfile_name='cam_desaxee')
        self.pb_cam_desaxee_launch.released.connect(lambda: self.launchCameraStream("cam_desaxee",
                                                                                     30,
                                                                                     cam_desaxee_interface,
                                                                                     self.pb_cam_desaxee_launch,
                                                                                     self.cb_cam_desaxee_resolution,
                                                                                     self.pb_current_cam_desaxee_resolution,
                                                                                     self.cb_cam_desaxee_framerate,
                                                                                     self.pb_current_cam_desaxee_framerate))
        self.pb_cam_desaxee_apply.released.connect(lambda: self.ApplyCameraStream("cam_desaxee",
                                                                                   30,
                                                                                   cam_desaxee_interface,
                                                                                   self.pb_cam_desaxee_launch,
                                                                                   self.cb_cam_desaxee_resolution,
                                                                                   self.pb_current_cam_desaxee_resolution,
                                                                                   self.cb_cam_desaxee_framerate,
                                                                                   self.pb_current_cam_desaxee_framerate))
        
        cam_bras_interface = LaunchInterface(uuid= uuid, pkg_name= 'rover_control', launchfile_name='cam_bras')
        self.pb_cam_bras_launch.released.connect(lambda: self.launchCameraStream("cam_bras",
                                                                                     30,
                                                                                     cam_bras_interface,
                                                                                     self.pb_cam_bras_launch,
                                                                                     self.cb_cam_bras_resolution,
                                                                                     self.pb_current_cam_bras_resolution,
                                                                                     self.cb_cam_bras_framerate,
                                                                                     self.pb_current_cam_bras_framerate))
        self.pb_cam_bras_apply.released.connect(lambda: self.ApplyCameraStream("cam_bras",
                                                                                   30,
                                                                                   cam_bras_interface,
                                                                                   self.pb_cam_bras_launch,
                                                                                   self.cb_cam_bras_resolution,
                                                                                   self.pb_current_cam_bras_resolution,
                                                                                   self.cb_cam_bras_framerate,
                                                                                   self.pb_current_cam_bras_framerate))


        cam_pelle_interface = LaunchInterface(uuid= uuid, pkg_name= 'rover_control', launchfile_name='cam_pelle')
        self.pb_cam_pelle_launch.released.connect(lambda: self.launchCameraStream("cam_pelle",
                                                                                     30,
                                                                                     cam_pelle_interface,
                                                                                     self.pb_cam_pelle_launch,
                                                                                     self.cb_cam_pelle_resolution,
                                                                                     self.pb_current_cam_pelle_resolution,
                                                                                     self.cb_cam_pelle_framerate,
                                                                                     self.pb_current_cam_pelle_framerate))
        self.pb_cam_pelle_apply.released.connect(lambda: self.ApplyCameraStream("cam_pelle",
                                                                                   30,
                                                                                   cam_pelle_interface,
                                                                                   self.pb_cam_pelle_launch,
                                                                                   self.cb_cam_pelle_resolution,
                                                                                   self.pb_current_cam_pelle_resolution,
                                                                                   self.cb_cam_pelle_framerate,
                                                                                   self.pb_current_cam_pelle_framerate))
        
        cam_base_bras_interface = LaunchInterface(uuid= uuid, pkg_name= 'rover_control', launchfile_name='cam_base_bras')
        self.pb_cam_base_bras_launch.released.connect(lambda: self.launchCameraStream("cam_base_bras",
                                                                                     30,
                                                                                     cam_base_bras_interface,
                                                                                     self.pb_cam_base_bras_launch,
                                                                                     self.cb_cam_base_bras_resolution,
                                                                                     self.pb_current_cam_base_bras_resolution,
                                                                                     self.cb_cam_base_bras_framerate,
                                                                                     self.pb_current_cam_base_bras_framerate))
        self.pb_cam_base_bras_apply.released.connect(lambda: self.ApplyCameraStream("cam_base_bras",
                                                                                   30,
                                                                                   cam_base_bras_interface,
                                                                                   self.pb_cam_base_bras_launch,
                                                                                   self.cb_cam_base_bras_resolution,
                                                                                   self.pb_current_cam_base_bras_resolution,
                                                                                   self.cb_cam_base_bras_framerate,
                                                                                   self.pb_current_cam_base_bras_framerate))
        
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
                rosparam.set_param('/' + cameraName + '/hardware/framerate', str(cameraFramerate))
                framerate = int(cb_framerate.currentText())
                param_skip = int(cameraFramerate/framerate)-1 
                rosparam.set_param(cameraName + '/compressed_packet_controller/skip', str(param_skip))
                
                if ((param_skip + 1) * framerate != cameraFramerate):
                    rospy.logerr(rospy.get_name() + "(" + self.name + "): " + "Wrong framerate parameters node can't start")
                    return

                try:
                    resolution = cb_resolution.currentText()
                except KeyError:
                    rospy.logerr(rospy.get_name() + "(" + self.name + "): " + "Selected resolution isn't defined ")
                    return
                
                param_imageWidth = self.resolutionDict[resolution][0]
                param_imageHeight = self.resolutionDict[resolution][1]
                rosparam.set_param('/' + cameraName + '/hardware/image_width', param_imageWidth)
                rosparam.set_param('/' + cameraName + '/hardware/image_height', param_imageHeight)
                
                rospy.logwarn(rospy.get_name() + "(" + self.name + "): " + "Selected resolution: " + resolution + "(" + param_imageWidth + "x" + param_imageHeight + ")")
                rospy.logwarn(rospy.get_name() + "(" + self.name + "): " + "selected framerate: " + str(framerate))

                pb_current_framerate.setText("Fps: " + str(framerate))
                pb_current_resolution.setText("Res: " + resolution)
            else:
                pb_current_framerate.setText("")
                pb_current_resolution.setText("")


            self.mainWindowsClass.launchFile(launch_interface, button)

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
            rospy.logwarn(rospy.get_name() + "(" + self.name + "): " + cameraName + " is not started yet")    


    def bandwidthInfoUpdate(self, timer_obj: rospy.Timer, bandwidth_interface: BandwidthInterface, pb_current_bw: QPushButton):
        bandwidth_interface.get_bw();
                                              # No news msgs in last 2 seconds
        if bandwidth_interface.bw == None or bandwidth_interface.tn == None or bandwidth_interface.t0 == None:
            if not bandwidth_interface._flag:
                pb_current_bw.setText("")
                rospy.logdebug(rospy.get_name() + "(" + self.name + "): " + "Topic: " + bandwidth_interface.topic_name + " not published yet")
                bandwidth_interface._flag = True
        elif (bandwidth_interface.tn-bandwidth_interface.t0) > bandwidth_interface.window:
            if not bandwidth_interface._flag:
                pb_current_bw.setText("")
                rospy.logdebug(rospy.get_name() + "(" + self.name + "): " + "Topic " + bandwidth_interface.topic_name + " stopped")
                bandwidth_interface._flag = True
        else:
            bandwidth_interface._flag = False
            pb_current_bw.setText(str(bandwidth_interface.bw) + "/s")