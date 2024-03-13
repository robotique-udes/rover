from PyQt5.QtWidgets import QWidget, QLabel
from PyQt5.QtGui import QPixmap
from PyQt5 import uic

class Home(QWidget):
    def __init__(self):
        super(Home,self).__init__()
        uic.loadUi("./ui/pages/home.ui", self)

        pixmap = QPixmap('./static/icons/logo.png')

        self.lb_logo : QLabel
        self.lb_logo = self.findChild(QLabel, "lb_logo")

        self.lb_logo.setPixmap(pixmap)
        self.resize(pixmap.width(), pixmap.height())