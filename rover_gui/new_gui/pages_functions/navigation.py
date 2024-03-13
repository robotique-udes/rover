from PyQt5.QtWidgets import QWidget
from PyQt5 import uic

class Navigation(QWidget):
    def __init__(self):
        super(Navigation,self).__init__()
        uic.loadUi("./ui/pages/navigation.ui", self)