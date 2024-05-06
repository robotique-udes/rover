import sys
import io
import math

from PyQt5 import QtCore
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication, QVBoxLayout, QWidget
from PyQt5.QtWebEngineWidgets import QWebEngineView
import folium
from folium.plugins import Realtime


class FoliumMapWidget(QWidget):
    def __init__(self, nav_widget):
        super().__init__()
        self.nav_widget = nav_widget
        self.markers = []  

        self.layout = QVBoxLayout()
        self.setLayout(self.layout)

        coordinate = (45.378511, -71.924161)
        coordinate2 = (45.478511, -71.424161)
        
        self.m = folium.Map(
            title='Test map',
            zoom_start=18.5,
            location=coordinate,
        )

        kw = {"prefix": "fa", "color": "green", "icon": "arrow-up"}
        angle = 180
        icon = folium.Icon(angle=angle, **kw)
        self.rover_marker = folium.Marker(location=coordinate, icon=icon, tooltip=str(angle))
        self.rover_marker.add_to(self.m)
        self.markers.append(self.rover_marker) 

        self.tile = folium.TileLayer(
            tiles = 'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
            attr = 'Esri',
            name = 'Esri Satellite',
            overlay = False,
            control = True,
            maxZoom = 25
        ).add_to(self.m)

        self.data = io.BytesIO()
        self.m.save(self.data, close_file=False)

        self.web_view = QWebEngineView()
        self.web_view.setHtml(self.data.getvalue().decode())

        self.web_view.setFixedSize(900, 900)

        self.layout.addWidget(self.web_view)

        self.update_locations()

    def update_locations(self):

        self.m = folium.Map(
            title='Test map',
            zoom_start=18.5,
            location=[45.378511, -71.924161],
        )
        self.tile.add_to(self.m)

        # Add markers from the nav_widget
        for i in range(len(self.nav_widget.locations)):
            marker = folium.Marker(
                location=[self.nav_widget.locations.iloc[i]['lat'], self.nav_widget.locations.iloc[i]['lon']],
                popup=self.nav_widget.locations.iloc[i]['name'],
                icon=folium.Icon(icon=str(self.nav_widget.locations.iloc[i]['index']), prefix='fa', color=self.nav_widget.locations.iloc[i]['color']),
            )
            marker.add_to(self.m)
            self.markers.append(marker)

        self.data = io.BytesIO()
        self.m.save(self.data, close_file=False)
        self.web_view.setHtml(self.data.getvalue().decode())

    def update_rover_location(self, new_lat, new_lon, new_heading):
        def update_rover_position():
            self.rover_marker.location = [new_lat, new_lon]
            self.rover_marker.tooltip = str(new_heading) 
            self.update_locations() 

        # Set up the Realtime plugin to call the update_rover_position function every second
        self.realtime = Realtime(interval=1000, callback=update_rover_position)
        self.realtime.add_to(self.m)

        print("Rover location updated:", new_lat, new_lon, "Heading:", new_heading)
