from ament_index_python.packages import get_package_share_directory

from PyQt5 import uic
from PyQt5.QtWidgets import QWidget

class Cameras(QWidget):
    def __init__(self, ui_node):
        super(Cameras,self).__init__()

        self.ui_node = ui_node

        package_share_directory = get_package_share_directory('rover_gui')
        uic.loadUi(package_share_directory+ "/ui/cameras.ui", self)

    def __del__(self):
        return