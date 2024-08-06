from ament_index_python.packages import get_package_share_directory

from PyQt5.QtWidgets import QWidget, QPushButton, QLineEdit, QComboBox
from PyQt5 import uic

from navigation.route_manager import Location

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

        self.pb_add_location.clicked.connect(self.add_location)
        self.pb_cancel.clicked.connect(self.cancel)

        if is_record == True:
            self.le_latitude.setText(str(self.nav_widget.current_latitude))
            self.le_longitude.setText(str(self.nav_widget.current_longitude))
            self.le_latitude.setEnabled(False)
            self.le_longitude.setEnabled(False)

    def add_location(self, button: QPushButton):

        self.nav_widget.route_manager.add_location_to_route(Location(self.le_name.text(),
                                                                    float(self.le_latitude.text()),
                                                                    float(self.le_longitude.text())))

        self.nav_widget.update_waypoint_list()
        self.deleteLater()
    
    def cancel(self):
        print("Location 1 " + str(self.locations[0]["name"]))