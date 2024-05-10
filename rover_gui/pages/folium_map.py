from threading import Thread
import time
import io
from PyQt5.QtWidgets import QVBoxLayout, QWidget, QCheckBox
from PyQt5.QtCore import QTimer, QObject, pyqtSignal
from PyQt5.QtWebEngineWidgets import QWebEngineView
import folium

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

        self.start_zoom = 19
        #self.start_coordinate = (45.504678343997625, -73.61237975463509)
        self.start_coordinate = (45.378248113468025, -71.92403818175768)
        
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
            location=(self.nav_widget.current_latitude - 0, self.nav_widget.current_longitude + 0),
        )
        self.tile.add_to(self.m)

        # Add markers from the nav_widget
        for i in range(len(self.nav_widget.locations)):
            marker = folium.Marker(
                location=[self.nav_widget.locations.iloc[i]['lat'], self.nav_widget.locations.iloc[i]['lon']],
                popup=self.nav_widget.locations.iloc[i]['name'],
                icon=folium.Icon(icon=str(self.nav_widget.locations.iloc[i]['index']), prefix='fa', color=self.nav_widget.locations.iloc[i]['color'], shadow_size=(0,0)),
            )
            marker.add_to(self.m)
            self.markers.append(marker)
            self.update_rover_location(self.nav_widget.current_latitude, self.nav_widget.current_longitude, self.nav_widget.current_heading)

        self.data = io.BytesIO()
        self.m.save(self.data, close_file=False)
        self.update_signal.update_signal.emit()
        

    def update_rover_location(self, new_lat, new_lon, new_heading):
        kw = {"prefix": "fa", "color": "green", "icon": "car"}
        icon = folium.Icon(angle=int(new_heading), **kw, shadow_size=(0,0))
        self.rover_marker = folium.Marker(location=[new_lat, new_lon], icon=icon, tooltip=str(int(new_heading)))
        self.rover_marker.add_to(self.m)
        self.markers.append(self.rover_marker)

    def toggle_update_status(self, state):
        if state == 0:
            self.is_update_disabled = False
        else:
            self.is_update_disabled = True

    def update_ui(self):
        self.web_view.setHtml(self.data.getvalue().decode())

    def task_update_thread(self):
        while self.alive:
            time.sleep(5)
            if not self.is_update_disabled:
                self.update_locations()