from ament_index_python.packages import get_package_share_directory

from PyQt5.QtWidgets import QWidget, QRadioButton
from PyQt5 import uic

from rover_gui import dashboard_node

import rclpy
from rclpy.client import Client
from rclpy.node import Node
from rover_msgs.srv._antenna_arbitration import AntennaArbitration

class Dashboard(QWidget):
    def __init__(self, ui_node):
        super(Dashboard,self).__init__()
        self.ui_node = ui_node
        package_share_directory = get_package_share_directory('rover_gui')
        uic.loadUi(package_share_directory+ "/ui/dashboard.ui", self)

        self.rb_teleop = self.findChild(QRadioButton, "rb_teleop")
        self.rb_autonomus = self.findChild(QRadioButton, "rb_autonomus")
        self.rb_immobile = self.findChild(QRadioButton, "rb_immobile")

        self.rb_teleop.clicked.connect(self.radio_button_clicked)
        self.rb_autonomus.clicked.connect(self.radio_button_clicked)
        self.rb_immobile.clicked.connect(self.radio_button_clicked)
        
    
    def radio_button_clicked(self):

        antenna_client = self.ui_node.create_client(AntennaArbitration, '/base/antenna/set_arbitration')
        antenna_req = AntennaArbitration.Request()

        sender_rb = self.sender()
        if sender_rb == self.rb_teleop:
            antenna_req.arbitration = 0
            self.ui_node.get_logger().info("Autonomous selected")

        elif sender_rb == self.rb_autonomus:
            antenna_req.arbitration = 1
            self.ui_node.get_logger().info("Autonomus selected")
        elif sender_rb == self.rb_immobile:
            antenna_req.arbitration = 2
            self.ui_node.get_logger().info("Immobile selected")

        future = antenna_client.call_async(antenna_req)
        print(future.result())