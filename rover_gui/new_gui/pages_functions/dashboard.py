from PyQt5.QtWidgets import QWidget
from PyQt5 import uic

class Dashboard(QWidget):
    def __init__(self):
        super(Dashboard,self).__init__()
        uic.loadUi("./ui/pages/dashboard.ui", self)