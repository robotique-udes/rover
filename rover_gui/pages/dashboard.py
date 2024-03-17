from ament_index_python.packages import get_package_share_directory

from PyQt5.QtWidgets import QWidget
from PyQt5 import uic

class Dashboard(QWidget):
    def __init__(self):
        super(Dashboard,self).__init__()
        package_share_directory = get_package_share_directory('rover_gui')
        uic.loadUi(package_share_directory+ "/ui/dashboard.ui", self)