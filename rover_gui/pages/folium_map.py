import io
from PyQt5.QtWidgets import QVBoxLayout, QWidget
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

        self.start_zoom = 18
        self.start_coordinate = (45.378511, -71.924161)
        
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

        self.update_locations()

    def update_locations(self):
        self.m = folium.Map(
            title='Test map',
            zoom_start=self.start_zoom,
            location=self.start_coordinate,
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
            self.update_rover_location(self.nav_widget.current_latitude, self.nav_widget.current_longitude, self.nav_widget.current_heading)
        
        self.data = io.BytesIO()
        self.m.save(self.data, close_file=False)
        self.web_view.setHtml(self.data.getvalue().decode())

    def update_rover_location(self, new_lat, new_lon, new_heading):
        kw = {"prefix": "fa", "color": "green", "icon": "arrow-up"}
        icon = folium.Icon(angle=int(new_heading), **kw)
        self.rover_marker = folium.Marker(location=[new_lat, new_lon], icon=icon, tooltip=str(int(new_heading)))
        self.rover_marker.add_to(self.m)
        self.markers.append(self.rover_marker)