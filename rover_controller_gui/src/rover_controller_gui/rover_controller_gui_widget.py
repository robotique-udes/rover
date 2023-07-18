from __future__ import (print_function, absolute_import, division, unicode_literals)

import os
import rospkg
import rospy
import rosparam
import rostopic
import rosservice
from python_qt_binding import loadUi
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtWidgets import QLabel, QPushButton, QAbstractButton, QComboBox, QApplication
from robotnik_msgs.msg import inputs_outputs
from robotnik_msgs.srv import set_digital_output

from rover_control_msgs.srv import camera_control
from rover_control_msgs.srv import camera_controlRequest
from rover_control_msgs.srv import camera_controlResponse

# Styling "MACROS"
STYLE_DEFAULT = ""
STYLE_SELECTED = "color: white; background-color: green"
STYLE_DISABLE = "color: white; background-color: grey"
STYLE_LIMITING = "color: black; background-color: yellow"
STYLE_WARN = "color: black; background-color: rgb(255, 124, 0);"

# Constant
RELAY_BOARD_SERVICE_NAME = "/rly_08_node/set_digital_outputs"
RELAY_BOARD_MESSAGE_NAME = "/rly_08_node/status"

CAMERA_CONTROL_SERVICE_NAME = "/camera_control_server"

class RoverControllerGuiWidget(QtWidgets.QWidget):
    
    def __init__(self):
        super(RoverControllerGuiWidget, self).__init__()

        ui_file = os.path.join(rospkg.RosPack().get_path('rover_controller_gui'), 'resource', 'rover_controller_gui.ui')
        loadUi(ui_file, self)
        self.setObjectName('RoverControllerGuiWidget')

        #Lights
        self.pb_lights.released.connect(lambda: self.toggleLights(self.pb_lights, 1))
        self.light_status_updater = rospy.Timer(rospy.Duration(1.0), lambda x: self.updateLightStatus(self.light_status_updater, self.pb_lights, 1))

        #Aruco Marker live feed
        self.last_detected_marker = list()
        self.aruco_marker_updater = rospy.Timer(rospy.Duration(1.0), lambda x: self.updateDetectedArucoMarker(self.aruco_marker_updater, self.pb_aruco_marker))

    def updateLightStatus(self, timer_obj: rospy.Timer, button: QPushButton, output_index: int):
        if not self.checkIfServicePosted(RELAY_BOARD_SERVICE_NAME, button):
            return
        
        # Actual callback
        try:
            data: inputs_outputs = rospy.wait_for_message(RELAY_BOARD_MESSAGE_NAME, inputs_outputs, rospy.Duration(0.1))

            if data.digital_outputs[output_index]:
                button.setStyleSheet(STYLE_SELECTED)
            else:
                button.setStyleSheet(STYLE_DEFAULT)

        except:
            rospy.logwarn_throttle(10, "Error rover_launch_control can't get relay status")

    def toggleLights(self, button: QPushButton, index: int):
        QApplication.setOverrideCursor(QtCore.Qt.WaitCursor)
        try :
            rospy.wait_for_service(RELAY_BOARD_SERVICE_NAME, rospy.Duration(1.0))

            set_digital_output_service: set_digital_output = rospy.ServiceProxy(RELAY_BOARD_SERVICE_NAME, set_digital_output)
            
            data: inputs_outputs = rospy.wait_for_message(RELAY_BOARD_MESSAGE_NAME, inputs_outputs, rospy.Duration(1.0))
            if data.digital_outputs.digital_outputs[index] == True:
                set_digital_output_service(index, False)
            else:
                set_digital_output_service(index, True)

        except:
            rospy.logwarn("Error calling relay board service")
            button.setStyleSheet(STYLE_WARN)
        finally:
            QApplication.restoreOverrideCursor()

    def updateDetectedArucoMarker(self, timer_obj: rospy.Timer, button: QPushButton):
        if not self.checkIfServicePosted(CAMERA_CONTROL_SERVICE_NAME, button):
            return

        try:
            camera_control_service: rospy.ServiceProxy = rospy.ServiceProxy(CAMERA_CONTROL_SERVICE_NAME, camera_control)
            data: camera_controlResponse = camera_control_service(camera_controlRequest.CMD_DETECT_ARUCO)
            detected_aruco_marker = list(data.detected_aruco_marker)

            feedback: str = ""
            rospy.loginfo(str(detected_aruco_marker))
            for aruco_marker_ids in detected_aruco_marker:
                if aruco_marker_ids in self.last_detected_marker:
                    feedback = feedback + str(aruco_marker_ids) + " | "

            rospy.loginfo(feedback)

            if feedback != "":
                feedback = feedback[:-2]

            if button.styleSheet != STYLE_DEFAULT: 
                button.setStyleSheet(STYLE_DEFAULT)
            button.setText(feedback)

            self.last_detected_marker = detected_aruco_marker

        except:
            rospy.logwarn("Error while trying to get aruco markers")

    # Not locking way to check for service existing
    def checkIfServicePosted(self, service_name: str, button: QPushButton) -> bool:
        lst_services: list = rosservice.get_service_list()
        if service_name not in lst_services:
            button.setStyleSheet(STYLE_DISABLE)
            return False
        return True
