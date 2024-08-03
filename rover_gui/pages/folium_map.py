#https://www.kaggle.com/code/codebreaker619/introduction-to-folium

from threading import Thread
import time
import io
from PyQt5.QtWidgets import QVBoxLayout, QWidget
from PyQt5.QtCore import QObject, pyqtSignal
from PyQt5.QtWebEngineWidgets import QWebEngineView
import folium
import folium.plugins as plugins

class UpdateSignal(QObject):
    update_signal = pyqtSignal()

class FoliumMapWidget(QWidget):
    def __init__(self, nav_widget):
        super().__init__()
        self.nav_widget = nav_widget
        self.markers = []  

        self.update_signal = UpdateSignal()
        self.update_signal.update_signal.connect(self.update_ui)

        self.layout = QVBoxLayout()
        self.setLayout(self.layout)

        self.start_zoom = 18
        self.start_coordinate = (51.453979297112134, -112.7136912987049)
        #self.start_coordinate = (45.378248113468025, -71.92403818175768)
        
        self.m = folium.Map(
            title='Test map',
            zoom_start=self.start_zoom,
            location=self.start_coordinate,
        )

        self.tile = folium.TileLayer(
            tiles='https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
            attr='Esri',
            name='Esri Satellite',
            overlay=False,
            control=True,
            maxZoom=25
        ).add_to(self.m)

        self.data = io.BytesIO()
        self.m.save(self.data, close_file=False)

        self.web_view = QWebEngineView()
        self.web_view.setHtml(self.data.getvalue().decode())
        self.web_view.setFixedSize(900, 900)

        self.layout.addWidget(self.web_view)

        self.nav_widget.cb_stop_updating.stateChanged.connect(self.toggle_update_status)
        self.update_locations()

        self.alive = True
        self.is_update_disabled = False
        self.map_update_thread = Thread(name="map_update_thread", target=self.task_update_thread, daemon=True)
        self.map_update_thread.start()

    def update_locations(self):
        self.m = folium.Map(
            title='Test map',
            zoom_start=self.start_zoom,
            location=(self.nav_widget.current_latitude, self.nav_widget.current_longitude),
        )
        self.tile.add_to(self.m)

        if self.nav_widget.route_manager.current_route:
            for index, waypoint in enumerate(self.nav_widget.route_manager.current_route.waypoints):
                if waypoint == self.nav_widget.route_manager.current_route.waypoints[-1]:
                    goal_marker = folium.Marker(
                        location = [waypoint.latitude, waypoint.longitude],
                        popup = "Goal",
                        icon=folium.Icon(icon="bullseye", prefix='fa', color='green', shadow_size=(0,0))
                    ).add_to(self.m)
                    self.markers.append(goal_marker)

                else:
                
                    checkpoint_marker = folium.Marker(
                        location=[waypoint.latitude, waypoint.longitude],
                        icon=plugins.BeautifyIcon(
                                border_color="#00ABDC",
                                text_color="#000000",
                                number=index,
                                inner_icon_style="margin-top:0;",
                            )
                    ).add_to(self.m)

                self.markers.append(checkpoint_marker)
                
        self.update_rover_location(self.nav_widget.current_latitude, self.nav_widget.current_longitude, self.nav_widget.current_heading)
        self.data = io.BytesIO()
        self.m.save(self.data, close_file=False)
        self.update_signal.update_signal.emit()
        

    def update_rover_location(self, new_lat, new_lon, new_heading):
        # Define the arrow icon using CSS and HTML
        arrow_icon_html = f"""
        <div style="
            transform: rotate({new_heading}deg);
            font-size: 24px;
            color: red;
            text-align: center;
            line-height: 24px;
        ">
            &#11014;
        </div>
        """
        # Create a DivIcon with the arrow
        arrow_icon = folium.DivIcon(html=arrow_icon_html)
        
        # Add the new rover marker
        self.rover_marker = folium.Marker(
            location=[new_lat, new_lon],
            icon=arrow_icon,
            tooltip=str(int(new_heading))
        )
        self.rover_marker.add_to(self.m)
        self.markers.append(self.rover_marker)
        
        # Save the map and emit the update signal
        self.data = io.BytesIO()
        self.m.save(self.data, close_file=False)
        self.update_signal.update_signal.emit()

    def toggle_update_status(self, state):
        if state == 0:
            self.is_update_disabled = False
        else:
            self.is_update_disabled = True

    def update_ui(self):
        self.web_view.setHtml(self.data.getvalue().decode())

    def task_update_thread(self):
        while self.alive:
            time.sleep(3)
            if not self.is_update_disabled:
                self.update_locations()

    def __del__(self):
        self.map_update_thread.join()