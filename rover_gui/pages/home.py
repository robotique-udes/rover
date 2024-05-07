from ament_index_python.packages import get_package_share_directory

from PyQt5.QtWidgets import QWidget, QLabel
from PyQt5.QtGui import QPixmap
from PyQt5 import uic

class Home(QWidget):
    def __init__(self, ui_node):
        super(Home,self).__init__()
        package_share_directory = get_package_share_directory('rover_gui')
        uic.loadUi(package_share_directory+ "/ui/home.ui", self)

        pixmap = QPixmap(package_share_directory + '/images/logo.png')

        self.lb_logo : QLabel
        self.lb_logo = self.findChild(QLabel, "lb_logo")

        self.lb_logo.setPixmap(pixmap)
        self.resize(pixmap.width(), pixmap.height())