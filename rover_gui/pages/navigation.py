#ros2 topic pub /gps_data rover_msgs/msg/Gps '{latitude: 60.0, longitude: 50.0}'



from threading import Lock
from ament_index_python.packages import get_package_share_directory
from PyQt5.QtWidgets import QWidget, QPushButton
from PyQt5.QtGui import QPixmap, QTransform
from PyQt5 import QtCore, uic
from PyQt5.QtWidgets import QGraphicsScene
from rover_msgs.msg import Gps
from pages.waypoint_popup import WaypointPopup


class Navigation(QWidget):
    EARTH_RADIUS = 6371

    def __init__(self, ui_node):
        super(Navigation, self).__init__()
        self.ui_node = ui_node
        
        package_share_directory = get_package_share_directory('rover_gui')
        uic.loadUi(package_share_directory + "/ui/navigation.ui", self)

        map_image_path = package_share_directory + "/images/studio_map.png"
        rover_icon_path = package_share_directory + "/images/arrow_icon.png"

        self.scene: QGraphicsScene = QGraphicsScene(self)
        self.pb_add_waypoint : QPushButton

        self.rover_logo_size = 20
        self.rover_logo_pixmap: QPixmap = QPixmap(rover_icon_path).scaled(self.rover_logo_size, self.rover_logo_size, QtCore.Qt.KeepAspectRatio)
        self.map_image = self.scene.addPixmap(QPixmap(map_image_path).scaled(640, 640, QtCore.Qt.KeepAspectRatio))

        self.lb_rover_icon.setPixmap(self.rover_logo_pixmap)
        self.lb_rover_icon.move(400, 100)

        self.top_left = ReferencePoint(self.frame.pos().x() + 5, self.frame.pos().y() + 22, 45.379355, -71.924921)
        self.bottom_right = ReferencePoint(self.frame.pos().x() + 644, self.frame.pos().y() + 590, 45.378290, -71.923240)
        self.offsetX = 0
        self.offsetY = 0

        self.lock_position: Lock = Lock()
        with self.lock_position:
            self.current_latitude: float = -690.0
            self.current_longitude: float = -690.0
            self.current_height: float = -690.0
            self.current_heading: float = -690.0

        self.gv_map.setScene(self.scene)
        
        self.pb_add_waypoint.clicked.connect(self.open_add_waypoint_popup)

        self.gps_sub = ui_node.create_subscription(
            Gps,
            '/rover/gps/position',
            self.gps_data_callback,
            1)

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
    
    def open_add_waypoint_popup(self):
        self.add_waypoint_popup = WaypointPopup()
        self.add_waypoint_popup.show()
        
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