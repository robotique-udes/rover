from ament_index_python.packages import get_package_share_directory
from threading import Lock
from PyQt5.QtWidgets import QWidget, QPushButton, QMessageBox, QListWidgetItem, QSlider
from PyQt5 import uic
from rover_msgs.srv._compass_calibration import CompassCalibration
from rover_msgs.msg import Gps
from rover_msgs.msg import Compass
from pages.add_location_popup import AddLocationPopup
from pages.calibrate_heading_popup import CalibrateHeadingPopup
from pages.folium_map import FoliumMapWidget
import pandas

class Navigation(QWidget):
    EARTH_RADIUS = 6371

    def __init__(self, ui_node):
        super(Navigation, self).__init__()
        self.ui_node = ui_node
        
        package_share_directory = get_package_share_directory("rover_gui")
        resources_directory = self.ui_node.get_resources_directory('rover_gui')
        uic.loadUi(resources_directory + "navigation.ui", self)

        self.saved_locations_path = package_share_directory + "/../../../../src/rover/rover_gui/saved_files/saved_locations.txt"
        self.gps_offset_path = package_share_directory + "/../../../../src/rover/rover_gui/saved_files/gps_offset.txt"

        self.slider_lat_offset : QSlider
        self.slider_lon_offset : QSlider
        self.pb_save_offset : QPushButton
        self.lat_offset = 0
        self.lon_offset = 0

        self.locations = pandas.DataFrame(columns=['index', 'name', 'lat', 'lon', 'color'])

        self.current_latitude = self.current_longitude = self.current_heading = -50.0

        self.folium_map_widget = FoliumMapWidget(self)
        self.verticalLayout.addWidget(self.folium_map_widget)

        self.lock_position = Lock()
        self.lock_orientation = Lock()
        self.update_position()
        self.update_orientation()

        self.pb_add_location.clicked.connect(lambda: self.open_add_location_popup(False))
        self.pb_delete_location.clicked.connect(self.delete_location)
        self.pb_update_location.clicked.connect(self.folium_map_widget.update_locations)
        self.pb_record_location.clicked.connect(lambda: self.open_add_location_popup(True))
        self.pb_save_offset.clicked.connect(lambda: self.save_offset)
        self.pb_calib_compass.clicked.connect(self.open_calibrate_heading_popup)
        self.slider_lat_offset.valueChanged.connect(self.offset_rover)
        self.slider_lon_offset.valueChanged.connect(self.offset_rover)

        self.gps_sub = ui_node.create_subscription(Gps, '/rover/gps/position', self.gps_data_callback, 1)
        self.compass_sub = ui_node.create_subscription(Compass, '/rover/auxiliary/compass', self.heading_callback, 1)
        
        self.load_gps_offset()
        self.load_locations()

    def handle_service_unavailability(self, sender_rb, service_name):
        sender_rb.setAutoExclusive(False)
        sender_rb.setChecked(False)
        sender_rb.setAutoExclusive(True)
        self.ui_node.get_logger().warn('%s service not available.' % service_name)
        QMessageBox.warning(self, "Service Not Available", "The %s service is not available." % service_name)
        
    def update_position(self): 
        with self.lock_position:
            self.lb_curr_position.setText(f"lat : {self.current_latitude:.6f}, lon : {self.current_longitude:.6f}")

    def update_orientation(self): 
        with self.lock_orientation:
            self.lb_curr_heading.setText(f"heading : {self.current_heading:.1f}°")
            
    def gps_data_callback(self, data: Gps):
        with self.lock_position:
            self.current_latitude = round(data.latitude, 6) + self.lat_offset
            self.current_longitude = round(data.longitude, 6) + self.lon_offset
        self.update_position()

    def heading_callback(self, data: Compass):
        with self.lock_orientation:
            self.current_heading = round(data.heading, 2)
            self.current_heading = data.heading
        self.update_orientation()

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
        self.add_location_popup = AddLocationPopup(self, self.ui_node, is_record)
        self.add_location_popup.show()

    def open_calibrate_heading_popup(self):
        self.calibrate_heading_popup = CalibrateHeadingPopup(self, self.ui_node)
        self.calibrate_heading_popup.show()

    def load_gps_offset(self):
        with open(self.gps_offset_path, "r") as f:
            for line in f:
                values = line.strip().split(";")
                if len(values) == 2:
                    self.lat_offset = float(values[0])
                    self.lon_offset = float(values[1])

                    self.slider_lat_offset.setValue(int(self.lat_offset * 100000))
                    self.slider_lon_offset.setValue(int(self.lon_offset * 100000))
                    return
        return None
    
    def offset_rover(self):
        sender = self.sender()
        if sender == self.slider_lat_offset:
            self.lat_offset = sender.value() / 100000
        else:
            self.lon_offset = sender.value() / 100000

        self.folium_map_widget.update_locations()

    def calibrate_heading(self, angle):
        sender_rb = self.sender()
        if sender_rb is None:
            return
        
        calibration_client = self.ui_node.create_client(CompassCalibration, '/rover/auxiliary/compass/calibrate')
        calibrate_req = CompassCalibration.Request()

        if not calibration_client.wait_for_service(timeout_sec=1.0):
            self.handle_service_unavailability(sender_rb, "compass_calibrator")
            return
        
        calibrate_req.angle_offset = int(angle)
        calibration_client.call(calibrate_req)

    def save_offset(self):
        with open(self.gps_offset_path, "w") as f:
            f.write(f"{self.lat_offset};{self.lon_offset}")

    def close_location_popup(self):
        self.add_location_popup.quit
        
    def closePopUp(self):
        self.hide()

    def __del__(self):
        self.ui_node.destroy_subscription(self.gps_sub)
        self.ui_node.destroy_subscription(self.compass_sub)
        self.alive = False

class ReferencePoint:
    def __init__(self, scrX, scrY, lat, lng):
        self.scrX = scrX
        self.scrY = scrY
        self.lat = lat
        self.lng = lng
