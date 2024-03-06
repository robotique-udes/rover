import os
import rclpy
from rclpy.node import Node

from PyQt5.QtWidgets import QMainWindow
from python_qt_binding import loadUi

import rospkg
from rover_msgs.msg import Gps

class MainWindow(QMainWindow):
    def __init__(self, node):
        super(MainWindow, self).__init__()
        self.node = node
        #ui_file = os.path.join(rospkg.RosPack().get_path('rover_gui'), 'resource', 'rover_ui.ui')
        loadUi('/home/nathan/ros2_ws/src/rover/rover_gui/resource/rover_ui.ui', self)

        #self.node.message_received.connect(self.update_label)

    def update_label(self):
        self.lb_curr_position.setText(str(self.number))

    def closeEvent(self, event):
        self.node.destroy_node()
        rclpy.shutdown()
        event.accept()