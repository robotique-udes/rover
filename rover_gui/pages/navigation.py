from ament_index_python.packages import get_package_share_directory
from threading import Lock
from PyQt5.QtWidgets import QWidget, QPushButton, QComboBox, QListWidget, QListWidgetItem, QLabel, QSlider
from PyQt5 import uic
from rover_msgs.srv._compass_calibration import CompassCalibration
from rover_msgs.msg import Gps
from rover_msgs.msg import Compass
from pages.add_location_popup import AddLocationPopup
from pages.calibrate_heading_popup import CalibrateHeadingPopup
from pages.folium_map import FoliumMapWidget
from navigation.route_manager import Route, Location, RouteManager

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

        self.lb_curr_position : QLabel
        self.lb_curr_heading : QLabel
        self.pb_add_waypoint : QPushButton
        self.pb_delete_waypoint : QPushButton
        self.pb_update_location : QPushButton
        self.pb_record_location : QPushButton
        self.waypoint_list : QListWidget
        self.cbox_routes : QComboBox
        self.pb_load_route : QPushButton

        self.slider_lat_offset : QSlider
        self.slider_lon_offset : QSlider
        self.pb_save_offset : QPushButton
        self.lat_offset = 0
        self.lon_offset = 0

        #self.locations = pandas.DataFrame(columns=['index', 'name', 'lat', 'lon', 'color'])
        self.route_manager = RouteManager(self.ui_node)
        self.route_manager.load_routes()
        self.loaded_route : Route

        self.current_latitude = 51.453979297112134
        self.current_longitude = -112.7136912987049
        self.current_heading = 45

        self.folium_map_widget = FoliumMapWidget(self)
        self.verticalLayout.addWidget(self.folium_map_widget)

        self.lock_position = Lock()
        self.lock_orientation = Lock()
        self.update_position()
        self.update_orientation()

        self.pb_add_waypoint.clicked.connect(lambda: self.open_add_location_popup(False))
        self.pb_delete_waypoint.clicked.connect(self.delete_location)
        self.pb_update_location.clicked.connect(self.folium_map_widget.update_locations)
        self.pb_record_location.clicked.connect(lambda: self.open_add_location_popup(True))
        self.pb_save_offset.clicked.connect(self.save_offset)
        self.pb_load_route.clicked.connect(self.load_route)
        self.pb_save_offset.clicked.connect(lambda: self.save_offset)
        self.pb_calib_compass.clicked.connect(self.open_calibrate_heading_popup)
        self.slider_lat_offset.valueChanged.connect(self.offset_rover)
        self.slider_lon_offset.valueChanged.connect(self.offset_rover)

        self.gps_sub = ui_node.create_subscription(Gps, '/rover/gps/position', self.gps_data_callback, 1)
        self.compass_sub = ui_node.create_subscription(Compass, '/rover/auxiliary/compass', self.heading_callback, 1)
        
        self.load_gps_offset()
        self.update_waypoint_list()
        self.update_routes_combobox()
        
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

    def update_waypoint_list(self):
        self.waypoint_list.clear()
        
        if self.route_manager.current_route:
            for index, waypoint in enumerate(self.route_manager.current_route.waypoints):
                item = QListWidgetItem(f"({index}) {waypoint.name}: ({waypoint.latitude:.6f}, {waypoint.longitude:.6f})")
                self.waypoint_list.addItem(item)
                self.folium_map_widget.update_locations()

    def update_routes_combobox(self):
        if len(self.route_manager.routes) != 0:
            self.ui_node.get_logger().info("Update CB")
            self.cbox_routes.clear()
            for route in self.route_manager.routes:
                self.cbox_routes.addItem(route.name)
                
    def load_route(self):
        self.route_manager.set_current_route(self.cbox_routes.currentText())
        self.update_waypoint_list()

    def delete_location(self):
        selected_items = self.waypoint_list.selectedItems()
        if not selected_items:
            return  

        for item in selected_items:
            index = self.waypoint_list.row(item)
            self.locations = self.locations.drop(index)
            self.waypoint_list.takeItem(index)

        self.locations.reset_index(drop=True, inplace=True)

        with open(self.saved_locations_path, "w") as f:
            f.truncate(0)
            for _, location in self.locations.iterrows():
                f.write(f"{location['index']};{location['name']};{location['lat']};{location['lon']};{location['color']}\n")  

        self.update_waypoint_list()

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
        self.route_manager.__del__()
        self.ui_node.destroy_subscription(self.gps_sub)
        self.ui_node.destroy_subscription(self.compass_sub)
        self.alive = False
