from ament_index_python.packages import get_package_share_directory

from PyQt5.QtWidgets import QWidget
from PyQt5 import uic

class Navigation(QWidget):
    def __init__(self):
        super(Navigation,self).__init__()
        package_share_directory = get_package_share_directory('rover_gui')
        uic.loadUi(package_share_directory+ "/ui/navigation.ui", self)