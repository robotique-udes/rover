#https://www.youtube.com/watch?v=jWxNfb7Hng8
from threading import Thread 
import sys
import signal

import PyQt5
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QTabWidget, QToolBox, QShortcut
from PyQt5.QtGui import QKeySequence, QIcon
from PyQt5 import QtGui
from PyQt5 import uic
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rover_gui.ui_node import UINode

from pages.home import Home
from pages.dashboard import Dashboard
from pages.navigation import Navigation
from pages.cameras import Cameras

from static.resource_rc import qt_resource_data

class MainWindow(QMainWindow):
    def __init__(self, ui_node, executor):
        super(MainWindow, self).__init__()
        self.ui_node = ui_node
        self.executor = executor
        self.camera_window = None

        shortcut = QShortcut(QKeySequence("Ctrl+W"), self)
        shortcut.activated.connect(self.close_application)

        self.pages_created = [] 

        resources_directory = self.ui_node.get_resources_directory('rover_gui')
        uic.loadUi(resources_directory + "main_window.ui", self)

        self.pb_home = self.findChild(QPushButton, "pb_home")
        self.pb_dashboard = self.findChild(QPushButton, "pb_dashboard")
        self.pb_navigation = self.findChild(QPushButton, "pb_navigation")
        self.pb_cameras = self.findChild(QPushButton, "pb_cameras")
        self.tab_widget = self.findChild(QTabWidget, "tab_widget")
        self.tool_box = self.findChild(QToolBox, "tool_box")


        self.menu_btns_dict = {
            self.pb_home: lambda: self.create_page(Home),
            self.pb_dashboard: lambda: self.create_page(Dashboard),
            self.pb_navigation: lambda: self.create_page(Navigation),
            self.pb_cameras: self.show_camera_window
        }

        self.show_home_window()

        self.tab_widget.tabCloseRequested.connect(self.close_tab)

        self.pb_home.clicked.connect(self.show_selected_window)
        self.pb_dashboard.clicked.connect(self.show_selected_window)
        self.pb_navigation.clicked.connect(self.show_selected_window)
        self.pb_cameras.clicked.connect(self.show_selected_window)

    def create_page(self, class_name):
        obj = class_name(self.ui_node)  # Create an instance of the class
        self.pages_created.append(obj)  # Add the object to the list
        return obj

    def show_home_window(self):
        result = self.open_tab_flag(self.pb_home.text())
        self.set_btn_checked(self.pb_home)

        if result[0]:
            self.tab_widget.setCurrentIndex(result[1])
        else:
            tab_title = self.pb_home.text()
            curIndex = self.tab_widget.addTab(Home(self.ui_node), tab_title)
            self.tab_widget.setCurrentIndex(curIndex)
            self.tab_widget.setVisible(True)

    def show_selected_window(self):
        button = self.sender()

        if button == self.pb_cameras:
            self.show_camera_window()
        else:
            result = self.open_tab_flag(button.text())
            self.set_btn_checked(button)

            if(result[0]):
                self.tab_widget.setCurrentIndex(result[1])
            else: 
                tab_title = button.text()
                curIndex = self.tab_widget.addTab(self.menu_btns_dict[button](), tab_title)
                self.tab_widget.setCurrentIndex(curIndex)
                self.tab_widget.setVisible(True)

    def show_camera_window(self):
        if self.camera_window is None:
            self.camera_window = Cameras(self.ui_node)
        self.camera_window.show()
        self.camera_window.activateWindow()
        self.set_btn_checked(self.pb_cameras)

    def close_tab(self, index):
        obj = self.tab_widget.widget(index)

        if obj in self.pages_created:
            self.pages_created.remove(obj)
            obj.__del__()

        self.tab_widget.removeTab(index)
        if self.tab_widget.count() == 0:
            self.tool_box.setCurrentIndex(0)
            self.show_home_window()

    def set_btn_checked(self, btn):
        for button in self.menu_btns_dict.keys():
            if button != btn:
                button.setChecked(False)
            else:
                button.setChecked(True)

    def open_tab_flag(self, btn_text):
        open_tab_count = self.tab_widget.count()

        for i in range(open_tab_count):
            tab_title = self.tab_widget.tabText(i)
            if tab_title == btn_text:
                return True, i
            else:
                continue

        return False,

    def closeEvent(self, event):
            if self.camera_window:
                self.camera_window.close()
            self.close_application()

    def close_application(self):
        for obj in self.pages_created:
            obj.__del__()

        self.ui_node.destroy_node()
        self.executor.shutdown()
        QApplication.quit()

def get_package_share_directory_parent(package_name):
    package_share_directory = rclpy.get_package_share_directory(package_name)
    parent_directory = '/'.join(package_share_directory.split('/')[:-1]) + '/'
    return parent_directory

def main(args=None):
    rclpy.init(args=args)

    signal.signal(signal.SIGINT, signal.SIG_DFL)

    # Adds support for 4K screens
    if hasattr(PyQt5.QtCore.Qt, 'AA_EnableHighDpiScaling'):
        PyQt5.QtWidgets.QApplication.setAttribute(PyQt5.QtCore.Qt.AA_EnableHighDpiScaling, True)
    if hasattr(PyQt5.QtCore.Qt, 'AA_UseHighDpiPixmaps'):
        PyQt5.QtWidgets.QApplication.setAttribute(PyQt5.QtCore.Qt.AA_UseHighDpiPixmaps, True)

    ui_node = UINode()
    executor = MultiThreadedExecutor()
    executor.add_node(ui_node)

    app = QApplication(sys.argv)
    window = MainWindow(ui_node, executor)

    # Start the ROS2 node on a separate thread
    thread = Thread(target=executor.spin)
    thread.start()

    window.show()
    sys.exit(app.exec_())

    
if __name__ == '__main__':
    main()
