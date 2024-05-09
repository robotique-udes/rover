from PyQt5.QtWidgets import QWidget, QLabel
from PyQt5.QtGui import QPixmap
from PyQt5 import uic

class Home(QWidget):
    def __init__(self, ui_node):
        super(Home,self).__init__()
        resources_directory = ui_node.get_resources_directory('rover_gui')
        uic.loadUi(resources_directory+ "home.ui", self)
        pixmap = QPixmap(resources_directory + 'logo.png')

        self.lb_logo : QLabel
        self.lb_logo = self.findChild(QLabel, "lb_logo")

        self.lb_logo.setPixmap(pixmap)
        self.resize(pixmap.width(), pixmap.height())