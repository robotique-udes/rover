import time
from ament_index_python.packages import get_package_share_directory
from PyQt5.QtMultimedia import QMediaPlayer, QMediaContent
from PyQt5 import uic
from PyQt5.QtWidgets import QWidget, QLabel, QCheckBox, QPushButton, QMessageBox, QInputDialog, QLineEdit
from PyQt5.QtCore import QUrl, QTimer, QThread, pyqtSignal
from rover_msgs.srv import ScreenshotControl
from rover_msgs.srv._screenshot_control import ScreenshotControl

class Cameras(QWidget):
    def __init__(self, ui_node):
        super(Cameras, self).__init__()

        self.ui_node = ui_node
        self.statusThread = None

        self.resources_directory = self.ui_node.get_resources_directory('rover_gui')
        uic.loadUi(self.resources_directory + "cameras.ui", self)

        self.media_players = []
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
        self.cam1_screenshot.clicked.connect(lambda: self.cb_screenshot(1))
        self.cam2_screenshot.clicked.connect(lambda: self.cb_screenshot(2))
        self.cam3_screenshot.clicked.connect(lambda: self.cb_screenshot(3))
        self.cam4_screenshot.clicked.connect(lambda: self.cb_screenshot(4))

        self.start_feeds()
        
    def start_feeds(self):
        self.ip_list = self.load_file_into_list(self.resources_directory + "r1m_ips.txt")

        for i, ip in enumerate(self.ip_list):
            mediaPlayer = QMediaPlayer(None, QMediaPlayer.VideoSurface)
            video_output = getattr(self, f'cam{i+1}')
            refresh_button = getattr(self, f'cam{i+1}_refresh')

            mediaPlayer.setVideoOutput(video_output)
            mediaPlayer.setMedia(QMediaContent(QUrl(ip)))

            # Ensure video widget size is fixed
            video_output.setMinimumSize(860, 450)

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
        # Preserve the current media and URL
        current_url = mediaPlayer.media().canonicalUrl()
        mediaPlayer.setMedia(QMediaContent(current_url))
        mediaPlayer.play()

    def load_file_into_list(self, filename):
        with open(filename, 'r') as file:
            lines = [line.strip() for line in file.readlines() if line.strip()]
        return lines

    def cb_screenshot(self, camera_number):
        screenshot_client = self.ui_node.create_client(ScreenshotControl, '/rover/auxiliary/control_screenshot')
        
        # Wait for the service to become available with a timeout
        if not screenshot_client.wait_for_service(timeout_sec=1.0):
            QMessageBox.warning(self, "Service Unavailable", 
                                "The screenshot service is currently unavailable. Please try again later.")
            return

        # Prompt user for screenshot name
        screenshot_name, ok = QInputDialog.getText(self, "Screenshot Name", 
                                                   "Enter the name for your screenshot:",
                                                   QLineEdit.Normal, "")
        
        if ok and screenshot_name:
            screenshot_req = ScreenshotControl.Request()
            screenshot_req.start = True
            
            ip_list = self.ip_list
            if 1 <= camera_number <= len(ip_list):
                self.ui_node.get_logger().info("Ip to send")
                screenshot_req.ip_address = ip_list[camera_number - 1]
                screenshot_req.name = screenshot_name
            else:
                self.ui_node.get_logger().error(f"Invalid camera number: {camera_number}")
                return
            
            response = screenshot_client.call(screenshot_req)

    def screenshot_callback(self, future):
        try:
            response = future.result()
            if response.success:
                QMessageBox.information(self, "Screenshot Saved", 
                                        f"Screenshot has been saved successfully as {response.name}")
            else:
                QMessageBox.warning(self, "Screenshot Error", 
                                    "Failed to save the screenshot. Please try again.")
        except Exception as e:
            QMessageBox.warning(self, "Screenshot Error", f"An error occurred: {str(e)}")

    def __del__(self):
        for mediaPlayer in self.media_players:
            mediaPlayer.stop()
            mediaPlayer.setMedia(QMediaContent())  # Clear the media

        if self.status_thread is not None:
            self.status_thread.requestInterruption()
            self.status_thread.quit()
            self.status_thread.wait()

        self.media_players.clear()
        # Note: Removed the line with screenshot_client_client as it's not defined in the scope
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