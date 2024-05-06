from threading import Lock
from ament_index_python.packages import get_package_share_directory
from PyQt5.QtWidgets import QWidget, QPushButton, QVBoxLayout
from PyQt5.QtGui import QPixmap, QTransform
from PyQt5 import QtCore, uic
from PyQt5.QtWidgets import QGraphicsScene, QListWidget, QListWidgetItem
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

        rover_icon_path = package_share_directory + "/images/arrow_icon.png"
        self.saved_locations_path = package_share_directory + "/../../../../src/rover/rover_gui/log/saved_locations.txt"

        self.lb_curr_position : QLabel
        self.lb_curr_heading : QLabel
        self.pb_add_location : QPushButton
        self.pb_delete_location : QPushButton
        self.location_list : QListWidget

        self.locations = pandas.DataFrame({
            'index':[],
            'name':[],
            'lat':[],
            'lon':[],
            'color':[]
        }, dtype=str)

        # Create FoliumMapWidget instance
        self.folium_map_widget = FoliumMapWidget(self)
        self.verticalLayout.addWidget(self.folium_map_widget)

        self.lock_position: Lock = Lock()
        with self.lock_position:
            self.current_latitude: float = -690.0
            self.current_longitude: float = -690.0
            self.current_height: float = -690.0
            self.current_heading: float = -690.0

        self.pb_add_location.clicked.connect(self.open_location_popup)
        self.pb_delete_location.clicked.connect(self.delete_location)

        self.gps_sub = ui_node.create_subscription(
            Gps,
            '/rover/gps/position',
            self.gps_data_callback,
            1)

        self.location_list.clear()
        self.load_locations()
        

    def update_current_position(self): 
        with self.lock_position:

            self.lb_curr_position.setText("lat : " + str(round(self.current_latitude, 6)) + ", lon : " + str(round(self.current_longitude, 6)))
            self.lb_curr_heading.setText("heading : " + str(self.current_heading) + "°")

            print("update_current_position")
            
            self.folium_map_widget.update_rover_location(self.current_latitude, self.current_longitude, self.current_heading)
            
    
    def gps_data_callback(self, data: Gps):
        with self.lock_position:
            self.current_latitude = round(data.latitude, 6)
            self.current_longitude = round(data.longitude, 6)
            self.heading = data.heading
            self.height = data.height
        self.update_current_position()

    def load_locations(self):
        try:
            self.clear_locations_dataframe()
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
                lines = []

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

    def clear_locations_dataframe(self):
        self.locations = self.locations.drop(self.locations.index)
        
    def print_locations(self):
        print("Locations:")
        for index, location in self.locations.iterrows():
            print(f"Index: {location['index']}, Name: {location['name']}, Latitude: {location['lat']}, Longitude: {location['lon']}, Color: {location['color']}")

    def open_location_popup(self):
        self.add_location_popup = AddLocationPopup(self)
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