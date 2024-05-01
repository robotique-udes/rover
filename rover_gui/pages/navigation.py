from threading import Lock
from ament_index_python.packages import get_package_share_directory
from PyQt5.QtWidgets import QWidget, QPushButton, QVBoxLayout
from PyQt5.QtGui import QPixmap, QTransform
from PyQt5 import QtCore, uic
from PyQt5.QtWidgets import QGraphicsScene, QListWidget, QListWidgetItem
from rover_msgs.msg import Gps
from pages.add_location_popup import AddLocationPopup
from pages.folium_map import FoliumMapWidget



class Navigation(QWidget):
    EARTH_RADIUS = 6371

    def __init__(self, ui_node):
        super(Navigation, self).__init__()
        self.ui_node = ui_node
        
        package_share_directory = get_package_share_directory('rover_gui')
        uic.loadUi(package_share_directory + "/ui/navigation.ui", self)

        map_image_path = package_share_directory + "/images/studio_map.png"
        rover_icon_path = package_share_directory + "/images/arrow_icon.png"
        self.saved_locations_path = package_share_directory + "/../../../../src/rover/rover_gui/log/saved_locations.txt"

        self.scene: QGraphicsScene = QGraphicsScene(self)
        self.pb_add_location : QPushButton
        self.pb_delete_location : QPushButton
        self.location_list : QListWidget

        self.locations = []
        self.location_list.clear()
        self.load_locations()

        self.lock_position: Lock = Lock()
        with self.lock_position:
            self.current_latitude: float = -690.0
            self.current_longitude: float = -690.0
            self.current_height: float = -690.0
            self.current_heading: float = -690.0

        #self.gv_map.setScene(self.scene)
        
        self.pb_add_location.clicked.connect(self.open_location_popup)
        self.pb_delete_location.clicked.connect(self.delete_location)

        self.gps_sub = ui_node.create_subscription(
            Gps,
            '/rover/gps/position',
            self.gps_data_callback,
            1)


        # Create FoliumMapWidget instance
        self.folium_map_widget = FoliumMapWidget(ui_node)
        self.verticalLayout.addWidget(self.folium_map_widget)
        

    def update_current_position(self):
        with self.lock_position:
            pos = self.latlng_to_screenXY(self.current_latitude, self.current_longitude)

            self.lb_curr_position.setText("lat : " + str(round(self.current_latitude, 6)) + ", lon : " + str(round(self.current_longitude, 6)))
            self.lb_curr_heading.setText("heading : " + str(self.heading) + "°")

            if hasattr(self, 'lb_rover_icon') and self.lb_rover_icon:
                rotated_pixmap = self.rover_logo_pixmap.transformed(QTransform().rotate(self.heading))
                self.lb_rover_icon.setPixmap(rotated_pixmap)
                self.lb_rover_icon.move(round(pos["x"]) , round(pos["y"]))
                self.lb_rover_icon.show()
            else:
                self.ui_node.get_logger().warn("Rover icon is not initialized.")
    
    def gps_data_callback(self, data: Gps):
        with self.lock_position:
            self.current_latitude = round(data.latitude, 6)
            self.current_longitude = round(data.longitude, 6)
            self.heading = data.heading
            self.height = data.height
        self.update_current_position()
        
    def latlng_to_screenXY(self, lat, lng):
        perX = (lng - self.top_left.lng) / (self.bottom_right.lng - self.top_left.lng)
        perY = (lat - self.top_left.lat) / (self.bottom_right.lat - self.top_left.lat)

        scrX = self.top_left.scrX + (self.bottom_right.scrX - self.top_left.scrX) * perX
        scrY = self.top_left.scrY + (self.bottom_right.scrY - self.top_left.scrY) * perY

        return {'x': scrX, 'y': scrY}

    def load_locations(self):
        try:
            with open(self.saved_locations_path, "r") as f:
                for line in f:
                    parts = line.strip().split(";")
                    if len(parts) == 4: 
                        index, name, latitude, longitude = parts
                        location = {
                            "index": int(index),
                            "name": name,
                            "latitude": float(latitude),
                            "longitude": float(longitude)
                        }
                        self.locations.append(location)
                        item = QListWidgetItem(f"{location['name']}: ({location['latitude']}, {location['longitude']})")
                        self.location_list.addItem(item)
        except FileNotFoundError:
            with open(self.saved_locations_path, "w") as f:
                lines = []

    def delete_location(self):
        selected_items = self.location_list.selectedItems()
        if not selected_items:
            return  

        for item in selected_items:
            index = self.location_list.row(item)  
            del self.locations[index]  
            self.location_list.takeItem(index) 

        # Rewrite the locations to the file
        with open(self.saved_locations_path, "w") as f:
            for location in self.locations:
                f.write(f"{location['index']};{location['name']};{location['latitude']};{location['longitude']}\n")
            
    
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