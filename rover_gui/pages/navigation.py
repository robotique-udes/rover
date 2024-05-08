from threading import Lock
from ament_index_python.packages import get_package_share_directory
from PyQt5.QtWidgets import QWidget, QPushButton, QVBoxLayout, QListWidget, QListWidgetItem, QLabel
from PyQt5.QtGui import QPixmap
from PyQt5 import uic
from rover_msgs.msg import Gps
from pages.add_location_popup import AddLocationPopup
from pages.folium_map import FoliumMapWidget
import pandas

class Navigation(QWidget):
    EARTH_RADIUS = 6371

    def __init__(self, ui_node):
        super(Navigation, self).__init__()
        self.ui_node = ui_node
        
        package_share_directory = get_package_share_directory('rover_gui')
        uic.loadUi(package_share_directory + "/ui/navigation.ui", self)

        self.saved_locations_path = package_share_directory + "/../../../../src/rover/rover_gui/log/saved_locations.txt"
        self.recorded_locations_path = package_share_directory + "/../../../../src/rover/rover_gui/log/recorded_locations.txt"

        self.lb_curr_position = self.findChild(QLabel, 'lb_curr_position')
        self.lb_curr_heading = self.findChild(QLabel, 'lb_curr_heading')
        self.pb_add_location = self.findChild(QPushButton, 'pb_add_location')
        self.pb_delete_location = self.findChild(QPushButton, 'pb_delete_location')
        self.pb_update_location = self.findChild(QPushButton, 'pb_update_location')
        self.pb_record_location = self.findChild(QPushButton, 'pb_record_location')
        self.location_list = self.findChild(QListWidget, 'location_list')

        self.locations = pandas.DataFrame(columns=['index', 'name', 'lat', 'lon', 'color'])

        self.current_latitude = self.current_longitude = self.current_height = self.current_heading = -690.0

        self.folium_map_widget = FoliumMapWidget(self)
        self.verticalLayout.addWidget(self.folium_map_widget)

        self.lock_position = Lock()
        self.update_current_position()

        self.pb_add_location.clicked.connect(lambda: self.open_add_location_popup(False))
        self.pb_delete_location.clicked.connect(self.delete_location)
        self.pb_update_location.clicked.connect(self.folium_map_widget.update_locations)
        self.pb_record_location.clicked.connect(lambda: self.open_add_location_popup(True))

        self.gps_sub = ui_node.create_subscription(Gps, '/rover/gps/position', self.gps_data_callback, 1)

        self.load_locations()
        
    def update_current_position(self): 
        with self.lock_position:
            self.lb_curr_position.setText(f"lat : {self.current_latitude:.6f}, lon : {self.current_longitude:.6f}")
            self.lb_curr_heading.setText(f"heading : {self.current_heading}°")
            
    def gps_data_callback(self, data: Gps):
        with self.lock_position:
            self.current_latitude = round(data.latitude, 6)
            self.current_longitude = round(data.longitude, 6)
            self.current_heading = data.heading
            self.current_height = data.height
        self.update_current_position()

    def load_locations(self):
        try:
            self.locations = pandas.DataFrame(columns=['index', 'name', 'lat', 'lon', 'color'])
            self.location_list.clear()
            with open(self.saved_locations_path, "r") as f:
                for line in f:
                    parts = line.strip().split(";")
                    if len(parts) == 5: 
                        index, name, latitude, longitude, color = parts
                        location = {
                            "index": int(index),
                            "name": name,
                            "lat": float(latitude),
                            "lon": float(longitude),
                            "color": color
                        }
                        self.locations = self.locations._append(location, ignore_index=True)
                        item = QListWidgetItem(f" {location['index']} - {location['name']}: ({location['lat']}, {location['lon']})")
                        self.location_list.addItem(item)
                self.folium_map_widget.update_locations()

        except FileNotFoundError:
            with open(self.saved_locations_path, "w") as f:
                pass

    def delete_location(self):
        selected_items = self.location_list.selectedItems()
        if not selected_items:
            return  

        for item in selected_items:
            index = self.location_list.row(item)
            self.locations = self.locations.drop(index)
            self.location_list.takeItem(index)

        # Reset index
        self.locations.reset_index(drop=True, inplace=True)

        with open(self.saved_locations_path, "w") as f:
            f.truncate(0)
            for _, location in self.locations.iterrows():
                f.write(f"{location['index']};{location['name']};{location['lat']};{location['lon']};{location['color']}\n")  

        self.load_locations()

    def record_location(self):
        try:
            with open(self.recorded_locations_path, "r") as f:
                lines = f.readlines()
        except FileNotFoundError:
            lines = []
        with open(self.recorded_locations_path, "a") as f:
            index = len(lines) + 1 
            f.write(f"{index};{str(self.current_latitude)};{str(self.current_longitude)}\n")

    def open_add_location_popup(self, is_record):
        self.add_location_popup = AddLocationPopup(self, is_record)
        self.add_location_popup.show()

    def close_location_popup(self):
        self.add_location_popup.quit
        
    def closePopUp(self):
        self.hide()

    def __del__(self):
        self.ui_node.destroy_subscription(self.gps_sub)

class ReferencePoint:
    def __init__(self, scrX, scrY, lat, lng):
        self.scrX = scrX
        self.scrY = scrY
        self.lat = lat
        self.lng = lng