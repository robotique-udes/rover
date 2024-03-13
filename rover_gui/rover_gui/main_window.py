import os
import rclpy
from rclpy.node import Node

from PyQt5.QtWidgets import QMainWindow
from python_qt_binding import loadUi

from PyQt5 import QtCore
from PyQt5.QtGui import QPixmap
from PyQt5.QtWidgets import QPushButton, QLineEdit, QGraphicsScene

import rospkg
from rover_msgs.msg import Gps

class MainWindow(QMainWindow):

    def __init__(self, node):
        super(MainWindow, self).__init__()
        self.node = node
        #ui_file = os.path.join(rospkg.RosPack().get_path('rover_gui'), 'resource', 'rover_ui.ui')
        loadUi('/home/nathan/ros2_ws/src/rover/rover_gui/resource/rover_ui.ui', self)
        map_image_path = '/home/nathan/ros2_ws/src/rover/rover_gui/resource/studio_map.png'
        rover_logo_path = '/home/nathan/ros2_ws/src/rover/rover_gui/resource/arrow_icon.png'

        
        self.scene: QGraphicsScene = QGraphicsScene(self)
        self.lb_rover_icon: QPushButton
        self.lb_curr_position: QLineEdit
        self.lb_curr_xy_coord: QLineEdit

        # Load the rover icon with the size and position
        self.rover_logo_size = 25
        self.rover_logo_pixmap: QPixmap = QPixmap(rover_logo_path).scaled(self.rover_logo_size, self.rover_logo_size, QtCore.Qt.KeepAspectRatio)

        self.map_image = self.scene.addPixmap(QPixmap(map_image_path).scaled(800,800, QtCore.Qt.KeepAspectRatio))

        # Set the QPixmap to QLabel
        self.lb_rover_icon.setPixmap(self.rover_logo_pixmap)
        self.lb_rover_icon.move(730, 342)
        self.lb_rover_icon.setFixedWidth(600)
        self.lb_rover_icon.setFixedHeight(600)

        self.gv_map.setScene(self.scene)

        self.node.message_received.connect(self.update_label)

    def update_label(self, lat, lon):
        self.lb_curr_position.setText("Longitude: {}, Latitude: {}".format(lat, lon))
        self.lb_curr_position.setText(str(self.number))

    def closeEvent(self, event):
        self.node.destroy_node()
        rclpy.shutdown()
        event.accept()