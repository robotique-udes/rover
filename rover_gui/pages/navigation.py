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
    EARTH_RADIUS = 6371

    def __init__(self, ui_node):
        super(Navigation, self).__init__()
        self.ui_node = ui_node
        
        package_share_directory = get_package_share_directory('rover_gui')
        uic.loadUi(package_share_directory + "/ui/navigation.ui", self)

        map_image_path = package_share_directory + "/images/studio_map.png"
        rover_icon_path = package_share_directory + "/images/arrow_icon.png"

        self.scene: QGraphicsScene = QGraphicsScene(self)

        self.rover_logo_size = 20
        self.rover_logo_pixmap: QPixmap = QPixmap(rover_icon_path).scaled(self.rover_logo_size, self.rover_logo_size, QtCore.Qt.KeepAspectRatio)
        self.map_image = self.scene.addPixmap(QPixmap(map_image_path).scaled(700, 700, QtCore.Qt.KeepAspectRatio))

        self.lb_rover_icon.setPixmap(self.rover_logo_pixmap)
        self.lb_rover_icon.move(400, 100)

        self.top_left = ReferencePoint(43, 33, 45.379342, -71.924912)
        self.bottom_right = ReferencePoint(688, 628, 45.378358, -71.923317)
        self.offset = 25

        self.top_left.pos = self.latlng_to_global_XY(self.top_left.lat, self.top_left.lng)
        self.bottom_right.pos = self.latlng_to_global_XY(self.bottom_right.lat, self.bottom_right.lng)

        self.lock_position: Lock = Lock()
        with self.lock_position:
            self.current_latitude: float = -690.0
            self.current_longitude: float = -690.0
            self.current_height: float = -690.0
            self.current_heading: float = -690.0

        self.gv_map.setScene(self.scene)

        self.gps_sub = ui_node.create_subscription(
            Gps,
            '/rover/gps/position',
            self.gps_data_callback,
            1)

    def update_current_position(self):
        with self.lock_position:
            pos = self.latlng_to_screenXY(self.current_latitude, self.current_longitude)

            self.lb_curr_position.setText("lat : " + str(round(self.current_latitude, 4)) + ", lon : " + str(round(self.current_longitude, 4)))
            self.lb_curr_xy_coord.setText("XY : (" + str(round(pos["x"], 4)) + ", " + str(round(pos["y"], 4)) + ")")

            if hasattr(self, 'lb_rover_icon') and self.lb_rover_icon:
                #self.lb_rover_icon.move(43,33)
                self.lb_rover_icon.move(round(pos["x"]) - self.offset, round(pos["y"]) - self.offset)
                self.lb_rover_icon.show()
            else:
                print("Rover icon is not initialized.")
    
    def gps_data_callback(self, data: Gps):
        with self.lock_position:
            self.current_latitude = data.latitude
            self.current_longitude = data.longitude
            self.heading = data.heading_track
            self.height = data.height
        self.update_current_position()
        

    def latlng_to_global_XY(self, lat, lng):
        x = Navigation.EARTH_RADIUS*lng*math.cos((self.top_left.lat + self.bottom_right.lat)/2)
        y = Navigation.EARTH_RADIUS*lat
        return {'x': x, 'y': y}

    
    def latlng_to_screenXY(self, lat, lng):
        pos = self.latlng_to_global_XY(lat, lng)
        perX = ((pos['x'] - self.top_left.pos['x']) / (self.bottom_right.pos['x'] - self.top_left.pos['x']))
        perY = ((pos['y'] - self.top_left.pos['y']) / (self.bottom_right.pos['y'] - self.top_left.pos['y']))

        return {
            'x': self.top_left.scrX + (self.bottom_right.scrX - self.top_left.scrX) * perX,
            'y': self.top_left.scrY + (self.bottom_right.scrY - self.top_left.scrY) * perY
        }
    
    def closePopUp(self):
        self.hide()

class ReferencePoint:
    def __init__(self, scrX, scrY, lat, lng):
        self.scrX = scrX
        self.scrY = scrY
        self.lat = lat
        self.lng = lng