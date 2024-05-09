import time
from ament_index_python.packages import get_package_share_directory
from PyQt5.QtMultimedia import QMediaPlayer, QMediaContent
from PyQt5 import uic
from PyQt5.QtWidgets import QWidget, QTextEdit
from PyQt5.QtCore import QUrl, QTimer, QThread, pyqtSignal

class Cameras(QWidget):
    def __init__(self, ui_node):
        super(Cameras,self).__init__()

        self.ui_node = ui_node
        self.statusThread = None

        self.package_share_directory = get_package_share_directory('rover_gui')
        uic.loadUi(self.package_share_directory+ "/ui/cameras.ui", self)

        self.media_players = []
        self.labels = []

        self.start_feeds()
        
    def start_feeds(self):
        ip_list = self.load_file_into_list(self.package_share_directory+ "/resource/r1m_ips.txt")

        for i, ip in enumerate(ip_list):
            mediaPlayer = QMediaPlayer(None, QMediaPlayer.VideoSurface)

            video_output = getattr(self, f'cam{i+1}') 
            refresh_button = getattr(self, f'cam{i+1}_refresh')

            mediaPlayer.setVideoOutput(video_output) 
            #rtsp_url = "rtmp://192.168.0.49/live/test"
            rtsp_url = f"rtsp://{ip.strip()}:8554/main.264"
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
            while True:
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