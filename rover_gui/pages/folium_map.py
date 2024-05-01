import sys
import io

from PyQt5 import QtCore
from PyQt5.QtWidgets import QApplication, QVBoxLayout, QWidget
from PyQt5.QtWebEngineWidgets import QWebEngineView
import folium


class FoliumMapWidget(QWidget):
    def __init__(self, ui_node):
        super().__init__()
        self.ui_node = ui_node

        self.layout = QVBoxLayout()
        self.setLayout(self.layout)

        coordinate = (45.378511, -71.924161)
        char_noire = (45.378298, -71.924108)

        m = folium.Map(
            title='Test map',
            zoom_start=18.5,
            location=coordinate,
        )

        tile = folium.TileLayer(
            tiles = 'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
            attr = 'Esri',
            name = 'Esri Satellite',
            overlay = False,
            control = True,
            maxZoom = 25
        ).add_to(m)

        marker = folium.Marker(
            location=char_noire,
            popup='Your Marker Popup Text Here',
            icon=folium.Icon(color='red')
        ).add_to(m)

        data = io.BytesIO()
        m.save(data, close_file=False)

        self.web_view = QWebEngineView()
        self.web_view.setHtml(data.getvalue().decode())

        # Set the fixed size for the web view
        self.web_view.setFixedSize(900, 900)

        self.layout.addWidget(self.web_view)