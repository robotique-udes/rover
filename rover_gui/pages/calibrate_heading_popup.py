from ament_index_python.packages import get_package_share_directory

from PyQt5.QtWidgets import QWidget, QPushButton, QLineEdit, QMessageBox
from PyQt5 import uic

class CalibrateHeadingPopup(QWidget):
    def __init__(self, nav_widget, ui_node):
        super(CalibrateHeadingPopup, self).__init__()

        self.ui_node = ui_node
        self.nav_widget = nav_widget

        resources_directory = self.ui_node.get_resources_directory('rover_gui')
        uic.loadUi(resources_directory + "calibrate_heading_popup.ui", self)

        self.pb_calibrate : QPushButton
        self.pb_cancel : QPushButton
        self.le_angle : QLineEdit

        self.pb_calibrate.clicked.connect(self.calibrate)
        self.le_angle.returnPressed.connect(self.calibrate)
        self.pb_cancel.clicked.connect(self.cancel)

    def calibrate(self):
        angle = self.le_angle.text()
        try:
            angle_int = int(angle)
            self.nav_widget.calibrate_heading(angle_int)
            self.deleteLater()
        except ValueError:
            QMessageBox.critical(self, "Error", "Please enter a valid integer.", QMessageBox.Ok)

    def cancel(self):
        self.deleteLater()