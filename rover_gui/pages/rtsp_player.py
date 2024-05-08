import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QLineEdit, QPushButton
from PyQt5.QtMultimedia import QMediaPlayer, QMediaContent
from PyQt5.QtMultimediaWidgets import QVideoWidget
from PyQt5.QtCore import QUrl, Qt

class RTSPPlayer(QWidget):
    def __init__(self):
        super().__init__()

        self.startRTSPPlayer()
        
    
    def startRTSPPlayer(self):
        
        self.mediaPlayer = QMediaPlayer(None, QMediaPlayer.VideoSurface)

        self.videoWidget = QVideoWidget()  

        layout = QVBoxLayout()
        layout.addWidget(self.videoWidget)  
        self.setLayout(layout)

        self.mediaPlayer.setVideoOutput(self.videoWidget) 
        rtsp_url = "rtsp://192.168.144.25:8554/main.264" 
        self.mediaPlayer.setMedia(QMediaContent(QUrl(rtsp_url)))
        self.mediaPlayer.play()
