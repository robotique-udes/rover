from ament_index_python.packages import get_package_share_directory

from PyQt5.QtWidgets import QWidget, QPushButton, QLineEdit, QComboBox
from PyQt5 import uic

from navigation.location_manager import Location

class AddLocationPopup(QWidget):
    def __init__(self, nav_widget,ui_node, is_record):
        super(AddLocationPopup, self).__init__()

        self.ui_node = ui_node
        self.nav_widget = nav_widget

        package_share_directory = get_package_share_directory('rover_gui')
        resources_directory = self.ui_node.get_resources_directory('rover_gui')
        self.saved_locations_path = package_share_directory + "/../../../../src/rover/rover_gui/saved_files/saved_locations.txt"
        uic.loadUi(resources_directory + "add_location.ui", self)

        self.pb_add_location : QPushButton
        self.pb_cancel : QPushButton
        self.le_name : QLineEdit
        self.le_latitude : QLineEdit
        self.le_longitude : QLineEdit
        self.cb_color : QComboBox

        self.pb_add_location.clicked.connect(self.add_location)
        self.pb_cancel.clicked.connect(self.cancel)

        if is_record == True:
            self.le_latitude.setText(str(self.nav_widget.current_latitude))
            self.le_longitude.setText(str(self.nav_widget.current_longitude))
            self.le_latitude.setEnabled(False)
            self.le_longitude.setEnabled(False)

    def add_location(self, button: QPushButton):
        try:
            with open(self.nav_widget.saved_locations_path, "r") as f:
                lines = f.readlines()
        except FileNotFoundError:
            with open(self.saved_locations_path, "w") as f:
                lines = []
        with open(self.nav_widget.saved_locations_path, "a") as f:
            index = len(lines) + 1 
            f.write(f"{index};{self.le_name.text()};{self.le_latitude.text()};{self.le_longitude.text()};{self.cb_color.currentText()}\n")

        self.nav_widget.location_list.clear()
        self.nav_widget.load_locations()

        self.deleteLater()
    
    def cancel(self):
        print("Location 1 " + str(self.locations[0]["name"]))