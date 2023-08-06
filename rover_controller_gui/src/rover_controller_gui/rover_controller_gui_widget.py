from __future__ import (print_function, absolute_import, division, unicode_literals)

import os
import sys
import rospkg
import rospy
import rosservice
from python_qt_binding import loadUi
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtWidgets import QPushButton, QComboBox, QApplication, QDoubleSpinBox, QLineEdit, QWidget, QFileDialog
from robotnik_msgs.msg import inputs_outputs
from robotnik_msgs.srv import set_digital_output
from std_srvs.srv import SetBool
from datetime import datetime
from threading import Lock
import math

sys.path.insert(0, rospkg.RosPack().get_path('rover_launch_control'))
from src.rover_launch_control.rover_launch_control_widget import RoverLaunchControlWidget

from rover_control_msgs.srv import camera_control
from rover_control_msgs.srv import camera_controlRequest
from rover_control_msgs.srv import camera_controlResponse
from std_msgs.msg import Float32
from rover_control_msgs.msg import gps

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
SET_ARM_JOY_SERVICE_NAME = "/set_arm_joy"
FILE_NAME_RECORDED_POSITION = rospkg.RosPack().get_path('rover_control') + "/../saved_elements/recorded_position.txt"
FILE_NAME_SAVED_WAYPOINTS = rospkg.RosPack().get_path('rover_control') + "/../saved_elements/saved_waypoint.txt"

class RoverControllerGuiWidget(QtWidgets.QWidget):
    
    def __init__(self, context):
        # Type: QtGui.plugin_context.PluginContext
        self.context = context
        # Types definition
        self.cb_waypoint_label: QComboBox
        self.le_waypoint_label: QLineEdit
        self.dsb_latitude: QDoubleSpinBox
        self.dsb_longitude: QDoubleSpinBox
        self.pb_target_position: QPushButton
        self.pb_curr_position: QPushButton
        self.pb_curr_heading: QPushButton
        self.pb_target_heading: QPushButton
        
        self.name: str = "RoverControllerGuiWidget"
        super(RoverControllerGuiWidget, self).__init__()

        ui_file = os.path.join(rospkg.RosPack().get_path('rover_controller_gui'), 'resource', 'rover_controller_gui.ui')
        loadUi(ui_file, self)
        self.setObjectName('RoverControllerGuiWidget')

        # Launcher UI
        self.launcher_started: bool = False
        self.launcher_flag: bool = False
        self.rover_launch_control_widget = RoverLaunchControlWidget()
        self.pb_launcher_ui.released.connect(lambda: self.launcherPluginCallback(self.pb_launcher_ui))

        # Lights
        self.pb_lights.released.connect(lambda: self.toggleLights(self.pb_lights, 1))
        self.light_status_updater = rospy.Timer(rospy.Duration(1.0), lambda x: self.updateLightStatus(self.light_status_updater, self.pb_lights, 1))

        # Arm Joy
        self.pb_set_arm_joy.released.connect(lambda: self.changeArmJoy(self.pb_set_arm_joy))

        # Aruco Marker live feed
        self.last_detected_marker = list()
        self.aruco_marker_updater = rospy.Timer(rospy.Duration(1.0), lambda x: self.updateDetectedArucoMarker(self.aruco_marker_updater, self.pb_aruco_marker))

        # Calib
        self.pb_calib.released.connect(lambda: self.calibrateWholeArm(self.pb_calib, self.cb_calib_select, self.dsb_angle_calib))

        # Waypoint:
        self.waypoint_files_name: list[str] = [FILE_NAME_SAVED_WAYPOINTS, FILE_NAME_SAVED_WAYPOINTS]
        for file in self.waypoint_files_name: self.initFile(file) 

        self.waypoints = dict()
        self.pb_open_new_waypoint.released.connect(lambda: self.openPopup(WaypointPopUp))
        self.cb_waypoint_label.currentIndexChanged.connect(lambda: self.updateSelectedWaypoint(self.cb_waypoint_label))
        self.recorded_waypoint_index: int = 0
        self.pb_save_current.released.connect(lambda: self.recordWaypoint(self.pb_save_current))
        
        self.sub_gps = rospy.Subscriber("/gps_data", gps, self.cbGPSData)
        self.position_updater = rospy.Timer(rospy.Duration(0.50), lambda x: self.updateCurrentPosition(self.position_updater))

        self.lock_position: Lock = Lock()
        with self.lock_position:
            self.current_latitude: float = -690.0
            self.current_longitude: float = -690.0
            self.current_height: float = -690.0
            self.current_heading: float = -690.0

        self.pb_load_waypoint_file: QPushButton
        self.pb_load_waypoint_file.released.connect(lambda: self.loadWaypointsFromFile(self.pb_load_waypoint_file))

    # Timer Callback: Wait for relay board msg and update corresponding button's style 
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
            rospy.logwarn_throttle(10, rospy.get_name() + "|" + self.name + " " + "Error rover_launch_control can't get relay status")

    # Timer Callback: Update UI with correspond current position
    def updateCurrentPosition(self, timer_obj: rospy.Timer):
        with self.lock_position:
            self.pb_curr_position.setText(str(self.current_latitude) + ", " + str(self.current_longitude))
            self.pb_curr_heading.setText(str(self.current_heading) + "°")

            label: str = self.cb_waypoint_label.currentText()
            if self.cb_waypoint_label.currentText() in self.waypoints:
                Distance, Heading = self.coordinatesToHeadingAndDistance(self.current_latitude,
                                                             self.current_longitude,
                                                             self.waypoints[label][0],
                                                             self.waypoints[label][1])
                self.pb_target_heading.setText(str(round(Heading)) + "°")
            elif label != "Select a waypoint":
                rospy.logerr("Something wrong my guy")
                self.pb_target_heading.setText("Something wrong my guy")

    # Timer Callback: Ask ArucoMarker service if it sees any aruco marker and update ui corresponding field
    def updateDetectedArucoMarker(self, timer_obj: rospy.Timer, button: QPushButton):
        if not self.checkIfServicePosted(CAMERA_CONTROL_SERVICE_NAME, button):
            return

        try:
            camera_control_service: rospy.ServiceProxy = rospy.ServiceProxy(CAMERA_CONTROL_SERVICE_NAME, camera_control)
            data: camera_controlResponse = camera_control_service(camera_controlRequest.CMD_DETECT_ARUCO)
            detected_aruco_marker = list(data.detected_aruco_marker)

            feedback: str = ""
            for aruco_marker_ids in detected_aruco_marker:
                if aruco_marker_ids in self.last_detected_marker:
                    feedback = feedback + str(aruco_marker_ids) + " | "

            if feedback != "":
                feedback = feedback[:-2]

            if button.styleSheet != STYLE_DEFAULT: 
                button.setStyleSheet(STYLE_DEFAULT)
            button.setText(feedback)

            self.last_detected_marker = detected_aruco_marker

        except:
            rospy.logwarn(rospy.get_name() + "|" + self.name + " " + "Error while trying to get aruco markers")

    # Subscriber Callback: Update current position members with GPS data
    def cbGPSData(self, data: gps):
        with self.lock_position:
            self.current_latitude = data.latitude
            self.current_longitude = data.longitude
            self.heading = data.heading_track
            self.height = data.height

    # Button Callback: Adds (or remove) RoverLauncherGui from rqt but keeps 
    # object reference
    def launcherPluginCallback(self, button: QPushButton):
        if not self.launcher_started:
            self.context.add_widget(self.rover_launch_control_widget)
        else:
            self.context.remove_widget(self.rover_launch_control_widget)
            
        self.launcher_started = not self.launcher_started

    # Button Callback: Wait for realy board msg and switch the state accordingly
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

    # Button Callback: Toggle joy state
    def changeArmJoy(self, button: QPushButton):
        if not self.checkIfServicePosted(SET_ARM_JOY_SERVICE_NAME, button):
            button.setStyleSheet(STYLE_DISABLE)
            return
        
        try:
            rospy.wait_for_service(SET_ARM_JOY_SERVICE_NAME, rospy.Duration(1.0))
            
            if button.styleSheet() != STYLE_SELECTED:
                if(rospy.ServiceProxy(SET_ARM_JOY_SERVICE_NAME, SetBool)(True)):
                    button.setStyleSheet(STYLE_SELECTED)
            else:
                rospy.ServiceProxy(SET_ARM_JOY_SERVICE_NAME, SetBool)(False)
                button.setStyleSheet(STYLE_DEFAULT)

        except:
            rospy.logerr(rospy.get_name() + "(" + self.name + "): " + "Error changing joy demux target")
            button.setStyleSheet(STYLE_WARN)

    # Button Callback: Send calibrate signal to selected joint
    def calibrateJoint(self, button: QPushButton, selected_joint: QComboBox, angle: QDoubleSpinBox):
        pub: rospy.Subscriber = rospy.Publisher("/arm/" + selected_joint.currentText() + "/C", Float32, queue_size=1)     
        
        # Waits for pub to be registered
        while (pub.get_num_connections() < 1):
            pass
        pub.publish(float(angle.text()))

    def calibrateWholeArm(self, button: QPushButton, selected_joint: QComboBox, angle: QDoubleSpinBox):
        QApplication.setOverrideCursor(QtCore.Qt.WaitCursor)

        # Args are useless but nice and used for quick swapping callback from calib button
        timeout = rospy.Time.now() + rospy.Duration(5.0)

        for i in range (0, 4):
            pub: rospy.Subscriber = rospy.Publisher("/arm/j" + str(i) + "/C", Float32, queue_size=1)  
            while (pub.get_num_connections() < 1 and rospy.Time.now() < timeout):
                pass 

            if (i == 0):
                pub.publish(180.0)
            elif (i > 1):
                pub.publish(-90.0)
            else:
                pub.publish(0.0)

            if (rospy.Time.now() > timeout):
                rospy.logwarn("Timeout when trying to calib, make sure joints are running")
                button.setStyleSheet(STYLE_WARN)
            else:
                button.setStyleSheet(STYLE_DEFAULT)

            QApplication.restoreOverrideCursor()

    # Button Callback: Update UI with correspond waypoint of the specified ComboBox selected item
    def updateSelectedWaypoint(self, combo_box: QComboBox):
        if combo_box.currentText() in self.waypoints:
            text: str = str(round(self.waypoints[combo_box.currentText()][0], 4)) + ",  " + str(round(self.waypoints[combo_box.currentText()][1], 4))
        else:
            if (combo_box.currentText() == "Select a waypoint"):
                text: str = ("Select a waypoint")
            else:
                rospy.logwarn("\"" + combo_box.currentText() + "\" doesn't exist in current waypoint dictionnary")
                text: str = ("Error... How???")

        self.pb_target_position.setText(text)
    
    # Button Callback: Record a waypoint with current position and export it to a backup file in case of app crash
    def recordWaypoint(self, button: QPushButton):
        label: str = "Recorded #" + str(self.recorded_waypoint_index)
        labels = [self.cb_waypoint_label.itemText(i) for i in range(self.cb_waypoint_label.count())]
        labels.append("");
        if (label not in labels and "=" in label):
            button.setStyleSheet(STYLE_WARN)
            rospy.logwarn("Invalid or already existing label")
            return 

        with self.lock_position:
            self.waypoints[label] = (self.current_latitude, self.current_longitude)
        self.cb_waypoint_label.addItem(label)

        button.setStyleSheet(STYLE_DEFAULT)
        
        try:
            file = open(FILE_NAME_RECORDED_POSITION, "a")
            with self.lock_position:
                file.write(label + " " + str(self.current_latitude) + " " + str(self.current_latitude) + " " + str(datetime.now()) + "\n")
            file.close()
        except:
            rospy.logwarn(self.name + ": Error writing waypoint to file try again")
            button.setStyleSheet(STYLE_WARN)

        self.recorded_waypoint_index += 1

    # Helper: Check for service existing (not locking way)
    def checkIfServicePosted(self, service_name: str, button: QPushButton) -> bool:
        lst_services: list = rosservice.get_service_list()
        if service_name not in lst_services:
            button.setStyleSheet(STYLE_DISABLE)
            return False
        return True

    # Helper: Convert two gps position to heading
    def coordinatesToHeadingAndDistance(self, lat1, lon1, lat2, lon2):
        R = 6371e3
        lat1 = math.radians(lat1)
        lat2 = math.radians(lat2)
        lon1 = math.radians(lon1)
        lon2 = math.radians(lon2)
        
        # Heading (B)
        X = math.cos(lat2) * math.sin(lon2-lon1)
        Y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(lon2-lon1)
        Heading = math.degrees(math.atan2(X, Y))
        
        # Direction
        a = math.sin((lat2-lat1)/2) * math.sin((lat2-lat1)/2) + math.cos(lat1) * math.cos(lat2) * math.sin((lon2-lon1)/2) * math.sin((lon2-lon1)/2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        Distance = R * c # in metres
        
        return [Distance, Heading]

    # Helper: Find gps from another position with a distance and a heading
    def headingAndDistanceToCoordinates(self, D, B, lat1, lon1):
        rospy.logwarn("D: %d, B: %d, lat1: %d, lat2: %d")

        R = 6371e3
        lat1 = math.radians(lat1)
        lon1 = math.radians(lon1)

        lat2 = lat1 + D/R * math.cos(math.radians(B))
        a = math.log(math.tan(math.pi/4 + lat2/2)/math.tan(math.pi/4 + lat1/2))
        q = (lat2-lat1)/a
        lon2 = lon1 + D/R * math.sin(math.radians(B))/q
        
        lat2 = math.degrees(lat2)
        lon2 = math.degrees(lon2)
        
        return [lat2, lon2]

    # Helper: Read waypoint from waypoint file syntaxt exemple is in 
    def loadWaypointsFromFile(self, button: QPushButton):
        # Syntaxt help:
        # ======================================================================
        # = Help:
        # = 	Comments starts with '='
        # = 	Format of waypoint:
        # = 	"label" "latitude" "longitude" units are Decimal Degree
        # = Exemple:
        # = #69 69.000 -69.0000420
        # ======================================================================
        select_file_dialog: QFileDialog = QFileDialog()

        try:
            file_name: [str, str] = select_file_dialog.getOpenFileName(self, 'Open file', FILE_NAME_RECORDED_POSITION + "/../" ,"Text files (*.txt)")

            with open(file_name[0]) as openfileobject:
                line_counter: int = 0
                for line in openfileobject:
                    line_counter += 1
                    elements = line.split()
                    
                    # Skip comments, empty line, or empty string 
                    if line[0] == '=' or "" or len(elements) == 0: 
                        continue

                    if len(elements) != 3:
                        rospy.loginfo("See help below for syntaxt:")
                        self.printWaypointSyntaxtHelp(line, line_counter)
                        continue
                    else:
                        try:
                            self.waypoints[elements[0]] = (float(elements[1]), float(elements[2]))
                            self.cb_waypoint_label.addItem(elements[0])
                        except Exception as e:
                            rospy.loginfo("")

        except Exception as e: 
            rospy.logwarn("Loading waypoints failed with " + str(e))

    # Helper: Open a popup of another QWidget class
    def openPopup(self, class_type):
        self.popUp = class_type(self) 
        self.popUp.show()

    # Helper: Make waypoint file header
    def initFile(self, file_path: str):
        try:
            file = open(file_path, "a")

        except FileNotFoundError:
            rospy.logwarn("Error oppening file \"" + file_path + "\" \nTrying to make file...")
            try: 
                os.makedirs(os.path.dirname(file_path), exist_ok=True)

            except Exception as e:
                rospy.logwarn("Unknown execption exiting")
                raise e
            
            finally:
                rospy.logwarn("File created successfully!")
            
        except Exception as e:
            rospy.logwarn("Unknown Exception exiting")
            raise e

        file = open(file_path, "a")
        file.write("================================================================================\n")
        file.write("= " + str(datetime.now()) + "\n")
        file.write("================================================================================\n")
        file.close()
    
    # Helper: Print waypoint syntaxt help
    def printWaypointSyntaxtHelp(line: str, index: int):
        rospy.logwarn("Syntax error in waypoint at line #" + str(index) + ":")
        rospy.loginfo("Line is: " + line.strip())
        help: str = """
================================================================================
Help:
    Comments starts with \'=\'
    Format of waypoint:
\"label\" \"latitude\" \"longitude\" units are Decimal Degree

Exemple:
#69 69.420 -69.0000420
================================================================================"""

    # Called when program is exiting safely (used for backup file beautify) 
    def exitingSafely(self):
        for file in self.waypoint_files_name: 
            file = open(file, "a")
            file.write("\n\n")
            file.close()
        
# Qwidget for add waypoint popup
class WaypointPopUp(QtWidgets.QWidget):
    def __init__(self, mainWindowsClass: RoverControllerGuiWidget):
        super().__init__()
        ui_file = os.path.join(rospkg.RosPack().get_path('rover_controller_gui'), 'resource', 'save_waypoint.ui')
        loadUi(ui_file, self)
        self.setObjectName('Record Waypoint')

        self.mainWindowsClass = mainWindowsClass

        self.pb_add_new_waypoint: QPushButton
        self.pb_add_new_waypoint.released.connect(lambda: self.addWaypoint(self.pb_add_new_waypoint))
        self.pb_add_new_waypoint_shifted.released.connect(lambda: self.addShiftedWaypoint(self.pb_add_new_waypoint_shifted))
        self.cb_shifted_waypoint_type_selector: QComboBox

    def addWaypoint(self, button: QPushButton):
        label: str = self.le_waypoint_label.text()
        labels = [self.mainWindowsClass.cb_waypoint_label.itemText(i) for i in range(self.mainWindowsClass.cb_waypoint_label.count())]
        labels.append("");
        if (label in labels or "=" in label):
            self.mainWindowsClass.pb_open_new_waypoint.setStyleSheet(STYLE_WARN)
            rospy.logwarn("Invalid or already existing label")
        else:
            self.mainWindowsClass.waypoints[label] = (self.dsb_latitude.value(), self.dsb_longitude.value())
            self.mainWindowsClass.cb_waypoint_label.addItem(label)
            self.mainWindowsClass.pb_open_new_waypoint.setStyleSheet(STYLE_DEFAULT)
        
            try:
                file = open(FILE_NAME_SAVED_WAYPOINTS, "a")
                file.write(label + " " + str(self.dsb_latitude.value()) + " " + str(self.dsb_longitude.value()) + "\n")
                file.close()
            except:
                rospy.logwarn(self.name + ": Error writing waypoint to file try again")
                self.mainWindowsClass.pb_open_new_waypoint.setStyleSheet(STYLE_WARN)

        self.le_waypoint_label.setText("")
        self.closePopUp()

    def addShiftedWaypoint(self, button: QPushButton):
        label: str = self.le_shifted_waypoint_label.text()
        labels = [self.mainWindowsClass.cb_waypoint_label.itemText(i) for i in range(self.mainWindowsClass.cb_waypoint_label.count())]
        labels.append("");
        if (label in labels or "=" in label):
            self.mainWindowsClass.pb_open_new_waypoint.setStyleSheet(STYLE_WARN)
            rospy.logwarn("Invalid or already existing label")

        else:
            coordinate: list(float)
            if (self.cb_shifted_waypoint_type_selector.currentText() == "Current Position"):
                coordinate = self.mainWindowsClass.headingAndDistanceToCoordinates(self.dsb_distance.value(),
                                                                                   self.dsb_heading.value(),
                                                                                   self.mainWindowsClass.current_latitude,
                                                                                   self.mainWindowsClass.current_longitude)
            else:
                coordinate = self.mainWindowsClass.headingAndDistanceToCoordinates(self.dsb_distance.value(),
                                                                                   self.dsb_heading.value(),
                                                                                   self.dsb_latitude.value(),
                                                                                   self.dsb_longitude.value())
            self.mainWindowsClass.waypoints[label] = (coordinate[0], coordinate[1])
            self.mainWindowsClass.cb_waypoint_label.addItem(label)
            self.mainWindowsClass.pb_open_new_waypoint.setStyleSheet(STYLE_DEFAULT)
            
            try:
                file = open(FILE_NAME_SAVED_WAYPOINTS, "a")
                file.write(label + " " + str(self.dsb_latitude.value()) + " " + str(self.dsb_longitude.value()) + "\n")
                file.close()
            except:
                rospy.logwarn(self.name + ": Error writing waypoint to file try again")
                self.mainWindowsClass.pb_open_new_waypoint.setStyleSheet(STYLE_WARN)
                
            self.le_waypoint_label.setText("")
            self.closePopUp()

    def closePopUp(self):
        self.hide()
        del self
