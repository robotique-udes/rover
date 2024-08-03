import time
from ament_index_python.packages import get_package_share_directory
from PyQt5.QtMultimedia import QMediaPlayer, QMediaContent
from PyQt5 import uic
from PyQt5.QtWidgets import QWidget, QTextEdit, QComboBox, QLabel
from PyQt5.QtCore import QUrl, QTimer, QThread, pyqtSignal

class Cameras(QWidget):
    def __init__(self, ui_node):
        super(Cameras,self).__init__()

        self.ui_node = ui_node
        self.statusThread = None

        self.resources_directory = self.ui_node.get_resources_directory('rover_gui')
        uic.loadUi(self.resources_directory+ "cameras.ui", self)

        self.media_players = []
        self.labels = []

        self.cb_aruco1 : QComboBox
        self.cb_aruco2 : QComboBox
        self.cb_aruco3 : QComboBox
        self.cb_aruco4 : QComboBox
        self.lb_aruco1 : QLabel
        self.lb_aruco2 : QLabel
        self.lb_aruco3 : QLabel
        self.lb_aruco4 : QLabel

        self.lb_aruco1.hide()
        self.lb_aruco2.hide()
        self.lb_aruco3.hide()
        self.lb_aruco4.hide()

        self.start_feeds()
        
    def start_feeds(self):
        ip_list = self.load_file_into_list(self.resources_directory+ "r1m_ips.txt")

        for i, ip in enumerate(ip_list):
            mediaPlayer = QMediaPlayer(None, QMediaPlayer.VideoSurface)

            video_output = getattr(self, f'cam{i+1}') 
            refresh_button = getattr(self, f'cam{i+1}_refresh')

            mediaPlayer.setVideoOutput(video_output)
            # rtsp_url = f"rtsp://{ip.strip()}:8554/main.264"
            # rtsp_url = "rtsp://admin:admin@192.168.144.61:69/live"
            rtsp_url = "http://commondatastorage.googleapis.com/gtv-videos-bucket/sample/ElephantsDream.mp4"
            mediaPlayer.setMedia(QMediaContent(QUrl(rtsp_url)))

            refresh_button.clicked.connect(lambda checked, player=mediaPlayer: self.try_reconnect(player))

            self.media_players.append(mediaPlayer)
            self.labels.append(getattr(self, f'cam{i+1}_label'))
            
            mediaPlayer.play()
        
        self.status_thread = self.StatusThread(self, self.media_players, self.labels)
        self.status_thread.status_signal.connect(self.update_label)
        self.status_thread.start()

    def update_label(self, label, color):
        label.setStyleSheet(f'background-color: {color}')

    def try_reconnect(self, mediaPlayer):
        rtsp_url = mediaPlayer.media().canonicalUrl().toString()  
        mediaPlayer.setMedia(QMediaContent(QUrl(rtsp_url)))
        mediaPlayer.play()

    def load_file_into_list(self, filename):
        with open(filename, 'r') as file:
            lines = [line.strip() for line in file.readlines() if line.strip()]
        return lines

    def __del__(self):
        for mediaPlayer in self.media_players:
            mediaPlayer.stop()
            
        self.status_thread.quit()
        self.status_thread.wait()

    class StatusThread(QThread):
        status_signal = pyqtSignal(QTextEdit, str)

        def __init__(self, parent, media_players, labels):
            super().__init__()
            self.media_players = media_players
            self.labels = labels

        def run(self):
            while not self.isInterruptionRequested():
                for i, mediaPlayer in enumerate(self.media_players):
                    status = mediaPlayer.mediaStatus()
                    if status == QMediaPlayer.NoMedia or status == QMediaPlayer.InvalidMedia \
                    or status == QMediaPlayer.EndOfMedia or status == QMediaPlayer.UnknownMediaStatus:
                        self.status_signal.emit(self.labels[i], "red")
                    elif status == QMediaPlayer.LoadingMedia or status == QMediaPlayer.StalledMedia:
                        self.status_signal.emit(self.labels[i], "yellow")
                    else:
                        self.status_signal.emit(self.labels[i], "green")
                time.sleep(5)

    def __del__(self):
        # Stop all media players
        for mediaPlayer in self.media_players:
            mediaPlayer.stop()
            mediaPlayer.setMedia(QMediaContent())  # Clear the media

        # Stop and wait for the status thread to finish
        if self.status_thread is not None:
            self.status_thread.requestInterruption()
            self.status_thread.quit()
            self.status_thread.wait()

        # Clear the media players list
        self.media_players.clear()

        # Call the parent class destructor
        super().__del__()
            
