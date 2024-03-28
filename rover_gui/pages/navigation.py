#ros2 topic pub /gps_data rover_msgs/msg/Gps '{latitude: 60.0, longitude: 50.0}'


import math
import time
from threading import Lock
import rclpy
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from PyQt5.QtWidgets import QWidget, QRadioButton, QLineEdit
from PyQt5.QtGui import QPixmap
from PyQt5 import QtWidgets, QtCore, uic
from PyQt5.QtWidgets import QGraphicsScene
from rover_msgs.msg import Gps


class Navigation(QWidget):
    def __init__(self,ui_node):
        super(Navigation,self).__init__()

        self.ui_node = ui_node

        package_share_directory = get_package_share_directory('rover_gui')
        uic.loadUi(package_share_directory+ "/ui/navigation.ui", self)

        # Paths to UI and resource files
        map_image_path = package_share_directory + "/images/studio_map.png"
        rover_icon_path = package_share_directory + "/images/arrow_icon.png"

        # Initialize the graphics scene
        self.scene: QGraphicsScene = QGraphicsScene(self)
        self.lb_curr_position: QLineEdit
        self.lb_curr_xy_coord: QLineEdit

        # Load the rover icon with the size and position
        self.rover_logo_size = 25
        self.rover_logo_pixmap: QPixmap = QPixmap(rover_icon_path).scaled(self.rover_logo_size, self.rover_logo_size, QtCore.Qt.KeepAspectRatio)

        self.map_image = self.scene.addPixmap(QPixmap(map_image_path).scaled(700,700, QtCore.Qt.KeepAspectRatio))

        # Set the QPixmap to QLabel
        self.lb_rover_icon.setPixmap(self.rover_logo_pixmap)
        self.lb_rover_icon.move(730, 342)
        self.lb_rover_icon.setFixedWidth(600)
        self.lb_rover_icon.setFixedHeight(600)

        self.earth_radius = 6371

        # Calculate global X and Y for top-left reference point        
        self.p0 = ReferencePoint(38, -275, 45.379342, -71.924912)
        # Calculate global X and Y for bottom-right reference point
        self.p1 = ReferencePoint(735, 350, 45.378358, -71.923317)

        # Calculate global X and Y for top-left reference point
        self.p0.pos = self.latlngToGlobalXY(self.p0.lat, self.p0.lng)
        # Calculate global X and Y for bottom-right reference point
        self.p1.pos = self.latlngToGlobalXY(self.p1.lat, self.p1.lng)

        self.lock_position: Lock = Lock()
        with self.lock_position:
            self.current_latitude: float = -690.0
            self.current_longitude: float = -690.0
            self.current_height: float = -690.0
            self.current_heading: float = -690.0

        self.gv_map.setScene(self.scene)

        self.ui_node.create_subscription(Gps, "/gps_data", self.cbGPSData, 1)
        #self.position_updater = rclpy.timer(rclpy.duration(0.50), lambda x: self.updateCurrentPosition(self.position_updater))

    # Timer Callback: Update UI with correspond current position
    def updateCurrentPosition(self, timer_obj: rclpy.timer):
        with self.lock_position:
            pos = self.latlngToScreenXY(self.current_latitude, self.current_longitude)

            self.lb_curr_position.setText("lat : " + str(round(self.current_latitude, 4)) + ", lon : " + str(round(self.current_longitude, 4)))
            self.lb_curr_xy_coord.setText("XY : (" + str(round(pos["x"], 4)) + ", " + str(round(pos["y"], 4)) + ")")

            print('x_position : ', pos["x"])
            print('y_position : ', pos["y"])
            if hasattr(self, 'lb_rover_icon') and self.lb_rover_icon:
                self.lb_rover_icon.move(pos["x"], pos["y"])
                # Ensure the rover icon is visible on the map.
                self.lb_rover_icon.show()
            else:
                print("Rover icon is not initialized.")
    
    # Subscriber Callback: Update current position members with GPS data
    def cbGPSData(self, data: Gps):
        print("Msg received")
        with self.lock_position:
            self.current_latitude = data.latitude
            self.current_longitude = data.longitude
            self.heading = data.heading_track
            self.height = data.height
        

    #https://stackoverflow.com/questions/16266809/convert-from-latitude-longitude-to-x-y
    # This function converts lat and lng coordinates to GLOBAL X and Y positions
    def latlngToGlobalXY(self, lat, lng):
        # Calculates x based on cos of average of the latitudes
        x = self.earth_radius*lng*math.cos((self.p0.lat + self.p1.lat)/2)
        # Calculates y based on latitude
        y = self.earth_radius*lat
        return {'x': x, 'y': y}

    
    # This function converts lat and lng coordinates to SCREEN X and Y positions
    def latlngToScreenXY(self, lat, lng):
        # Calculate global X and Y for projection point
        pos = self.latlngToGlobalXY(lat, lng)
        # Calculate the percentage of Global X position in relation to total global width
        perX = ((pos['x']-self.p0.pos['x'])/(self.p1.pos['x'] - self.p0.pos['x']))
        # Calculate the percentage of Global Y position in relation to total global height
        perY = ((pos['y']-self.p0.pos['y'])/(self.p1.pos['y'] - self.p0.pos['y']))

        # Returns the screen position based on reference points
        return {
            'x': self.p0.scrX + (self.p1.scrX - self.p0.scrX)*perX,
            'y': self.p0.scrY + (self.p1.scrY - self.p0.scrY)*perY
        }
    
    def closePopUp(self):
        self.hide()
        # Consider not using `del self` unless absolutely necessary

class ReferencePoint:
    def __init__(self, scrX, scrY, lat, lng):
        self.scrX = scrX
        self.scrY = scrY
        self.lat = lat
        self.lng = lng