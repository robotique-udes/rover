import time
from ament_index_python.packages import get_package_share_directory
from threading import Lock
from PyQt5.QtMultimedia import QMediaPlayer, QMediaContent
from PyQt5 import uic
from PyQt5.QtWidgets import QWidget, QLabel, QCheckBox, QMessageBox
from PyQt5.QtCore import QUrl, QTimer, QThread, pyqtSignal
from rover_msgs.srv._rtsp_stream import RtspStream
from rover_msgs.msg._aruco import Aruco



class Cameras(QWidget):
    def __init__(self, ui_node):
        super(Cameras, self).__init__()

        self.ui_node = ui_node
        self.statusThread = None

        self.resources_directory = self.ui_node.get_resources_directory('rover_gui')
        uic.loadUi(self.resources_directory + "cameras.ui", self)

        self.media_players = []
        self.ips = []
        self.labels = []

        # Initialize UI components
        self.cb_aruco1: QCheckBox
        self.cb_aruco2: QCheckBox
        self.cb_aruco3: QCheckBox
        self.cb_aruco4: QCheckBox
        self.lb_aruco1: QLabel
        self.lb_aruco2: QLabel
        self.lb_aruco3: QLabel
        self.lb_aruco4: QLabel

        self.lb_aruco1.hide()
        self.lb_aruco2.hide()
        self.lb_aruco3.hide()
        self.lb_aruco4.hide()

        self.start_feeds()

        self.cb_aruco1.stateChanged.connect(self.aruco_clicked)
        self.cb_aruco2.stateChanged.connect(self.aruco_clicked)
        self.cb_aruco3.stateChanged.connect(self.aruco_clicked)
        self.cb_aruco4.stateChanged.connect(self.aruco_clicked)

        self.lock_aruco = Lock()
        self.update_aruco()

        self.aruco_sub = ui_node.create_subscription(Aruco, '/rover/auxiliary/aruco', self.aruco_data_callback, 1)
    
    def handle_service_unavailability(self, sender_rb, service_name):
        sender_rb.setAutoExclusive(False)
        sender_rb.setChecked(False)
        sender_rb.setAutoExclusive(True)
        self.ui_node.get_logger().warn('%s service not available.' % service_name)
        QMessageBox.warning(self, "Service Not Available", "The %s service is not available." % service_name)

    def aruco_clicked(self):
        sender_rb = self.sender()
        if sender_rb is None:
            return

        rtsp_client = self.ui_node.create_client(RtspStream, '/base/auxiliary/set_aruco_rtsp')
        rstp_req = RtspStream.Request()

        if not rtsp_client.wait_for_service(timeout_sec=1.0):
            self.handle_service_unavailability(sender_rb, "rtsp_stream")
            return

        if sender_rb == self.cb_aruco1:
            rstp_req.target_arbitration = self.ips[0]
        elif sender_rb == self.cb_aruco2:
            rstp_req.target_arbitration = self.ips[1]
        elif sender_rb == self.cb_aruco3:
            rstp_req.target_arbitration = self.ips[2]
        elif sender_rb == self.cb_aruco4:
            rstp_req.target_arbitration = self.ips[3]

        response = rtsp_client.call(rstp_req)

    def aruco_data_callback(self):
        with self.lock_position:
            self.current_latitude = round(data.latitude, 6) + self.lat_offset
            self.current_longitude = round(data.longitude, 6) + self.lon_offset
        self.update_position()

    def update_aruco_label(self):
        with self.lock_aruco:
            self.lb_curr_position.setText(f"lat : {self.current_latitude:.6f}, lon : {self.current_longitude:.6f}")
        
    def start_feeds(self):
        ip_list = self.load_file_into_list(self.resources_directory + "r1m_ips.txt")

        for i, ip in enumerate(ip_list):
            mediaPlayer = QMediaPlayer(None, QMediaPlayer.VideoSurface)
            video_output = getattr(self, f'cam{i+1}')
            refresh_button = getattr(self, f'cam{i+1}_refresh')

            mediaPlayer.setVideoOutput(video_output)
            #rtsp_url = "http://commondatastorage.googleapis.com/gtv-videos-bucket/sample/ElephantsDream.mp4"
            mediaPlayer.setMedia(QMediaContent(QUrl(ip)))

            # Ensure video widget size is fixed
            video_output.setMinimumSize(860, 450)

            refresh_button.clicked.connect(lambda checked, player=mediaPlayer: self.try_reconnect(player))

            self.media_players.append(mediaPlayer)
            self.ips.append(ip)
            self.labels.append(getattr(self, f'cam{i+1}_label'))
            
            mediaPlayer.play()
        
        self.status_thread = self.StatusThread(self, self.media_players, self.labels)
        self.status_thread.status_signal.connect(self.update_label)
        self.status_thread.start()

    def update_label(self, label, color):
        label.setStyleSheet(f'background-color: {color}')

    def try_reconnect(self, mediaPlayer):
        # Preserve the current media and URL
        current_url = mediaPlayer.media().canonicalUrl()
        mediaPlayer.setMedia(QMediaContent(current_url))
        mediaPlayer.play()

    def load_file_into_list(self, filename):
        with open(filename, 'r') as file:
            lines = [line.strip() for line in file.readlines() if line.strip()]
        return lines

    def __del__(self):
        for mediaPlayer in self.media_players:
            mediaPlayer.stop()
            mediaPlayer.setMedia(QMediaContent())  # Clear the media

        if self.status_thread is not None:
            self.status_thread.requestInterruption()
            self.status_thread.quit()
            self.status_thread.wait()

        self.media_players.clear()
        super().__del__()

    class StatusThread(QThread):
        status_signal = pyqtSignal(QLabel, str)

        def __init__(self, parent, media_players, labels):
            super().__init__()
            self.media_players = media_players
            self.labels = labels

        def run(self):
            while not self.isInterruptionRequested():
                for i, mediaPlayer in enumerate(self.media_players):
                    status = mediaPlayer.mediaStatus()
                    if status in [QMediaPlayer.NoMedia, QMediaPlayer.InvalidMedia, QMediaPlayer.EndOfMedia, QMediaPlayer.UnknownMediaStatus]:
                        self.status_signal.emit(self.labels[i], "red")
                    elif status in [QMediaPlayer.LoadingMedia, QMediaPlayer.StalledMedia]:
                        self.status_signal.emit(self.labels[i], "yellow")
                    else:
                        self.status_signal.emit(self.labels[i], "green")
                time.sleep(5)
