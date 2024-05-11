import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QLineEdit, QPushButton
from PyQt5.QtMultimedia import QMediaPlayer, QMediaContent
from PyQt5.QtMultimediaWidgets import QVideoWidget
from PyQt5.QtCore import QUrl, Qt

class RTSPPlayer(QWidget):
    def __init__(self, rtsp_url):
        super().__init__()

        self.rtsp_url = rtsp_url
        self.startRTSPPlayer()
        
    
    def startRTSPPlayer(self):
        
        self.mediaPlayer = QMediaPlayer(None, QMediaPlayer.VideoSurface)
        
        self.mediaPlayer.setVideoOutput(self.videoWidget) 
        self.mediaPlayer.setMedia(QMediaContent(QUrl(self.rtsp_url)))
        self.mediaPlayer.play()
