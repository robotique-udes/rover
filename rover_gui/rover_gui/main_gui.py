import sys
import rclpy
import threading
from .main_window import MainWindow
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QObject, pyqtSignal
from rclpy.node import Node

from rover_msgs.msg import Gps

class MainGui(Node, QObject):

    message_received = pyqtSignal(float, float)  # Define a signal

    def __init__(self):
        super().__init__("main_gui")
        self.float_topic_name = '/gps_talker'
        
        self.subscription = self.create_subscription(
            Gps,
            self.float_topic_name,
            self.sub_gps_callback,
            10,
        )
        self.subscription  # prevent unused variable warning
        
    ### ROS2 Data Updater
    def sub_gps_callback(self, msg):
        self.get_logger().info("Msg received : lon:%f lat:%f" % (msg.longitude, msg.latitude))
        self.message_received.emit(msg.latitude, msg.longitude)

def ros_spin(node):
    rclpy.spin(node)

def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    node = MainGui()

    main_window = MainWindow(node)
    main_window.show()

    ros_thread = threading.Thread(target=ros_spin, args=(node,))
    ros_thread.start()

    sys.exit(app.exec_())
    