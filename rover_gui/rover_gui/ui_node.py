import sys

import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QTabWidget, QToolBox
from rover_msgs.msg._joy_demux_status import JoyDemuxStatus


class UINode(Node):
    def __init__(self):
        super().__init__('ui_node')