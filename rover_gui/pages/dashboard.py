from ament_index_python.packages import get_package_share_directory

from PyQt5.QtWidgets import QWidget, QRadioButton, QMessageBox, QPushButton
from PyQt5.QtMultimedia import QMediaPlayer, QMediaContent
from PyQt5.QtCore import QUrl
from PyQt5 import uic
from rover_msgs.srv._antenna_arbitration import AntennaArbitration
from rover_msgs.srv._drive_train_arbitration import DriveTrainArbitration
from rover_msgs.srv._joy_demux_set_state import JoyDemuxSetState
from rover_msgs.srv._light_control import LightControl
from rover_msgs.msg._joy_demux_status import JoyDemuxStatus
from pages.rtsp_player import RTSPPlayer

class Dashboard(QWidget):
    def __init__(self, ui_node):
        super(Dashboard,self).__init__()
        self.ui_node = ui_node

        self.joydemux_sub = ui_node.create_subscription(
            JoyDemuxStatus,
            '/joy/demux/status',
            self.update_joydemux_button_status,
            1)

        resources_directory = self.ui_node.get_resources_directory('rover_gui')
        uic.loadUi(resources_directory+ "dashboard.ui", self)


        # Lights
        self.rb_normal_light : QPushButton
        self.rb_infrared_light : QPushButton

        # Joy arbitration ui elements
        self.rb_dt_none : QRadioButton
        self.rb_dt_teleop : QRadioButton
        self.rb_dt_autonomus : QRadioButton

        # Antenna arbitration ui elements
        self.rb_ant_none : QRadioButton
        self.rb_ant_teleop : QRadioButton
        self.rb_ant_autonomus : QRadioButton

        # Joy demux ui elements
        self.rb_joy_force : QRadioButton
        self.rb_joy_drivetrain_main : QRadioButton
        self.rb_joy_arm_main : QRadioButton
        self.rb_joy_antenna_main : QRadioButton
        self.rb_joy_none_main : QRadioButton
        self.rb_joy_drivetrain_sec : QRadioButton
        self.rb_joy_arm_sec : QRadioButton
        self.rb_joy_antenna_sec : QRadioButton
        self.rb_joy_none_sec : QRadioButton


        self.rb_ant_none.clicked.connect(self.antenna_arbitration_clicked)
        self.rb_ant_teleop.clicked.connect(self.antenna_arbitration_clicked)
        self.rb_ant_autonomus.clicked.connect(self.antenna_arbitration_clicked)
        self.rb_joy_drivetrain_main.clicked.connect(self.joydemux_clicked)
        self.rb_joy_antenna_main.clicked.connect(self.joydemux_clicked)
        self.rb_joy_arm_main.clicked.connect(self.joydemux_clicked)
        self.rb_joy_none_main.clicked.connect(self.joydemux_clicked)
        self.rb_joy_drivetrain_sec.clicked.connect(self.joydemux_clicked)
        self.rb_joy_antenna_sec.clicked.connect(self.joydemux_clicked)
        self.rb_joy_arm_sec.clicked.connect(self.joydemux_clicked)
        self.rb_joy_none_sec.clicked.connect(self.joydemux_clicked)
        self.rb_dt_none.clicked.connect(self.drivetrain_arbitration_clicked)
        self.rb_dt_teleop.clicked.connect(self.drivetrain_arbitration_clicked)
        self.rb_dt_autonomus.clicked.connect(self.drivetrain_arbitration_clicked)
        self.rb_normal_light.clicked.connect(self.light_mode_clicked)
        self.rb_infrared_light.clicked.connect(self.light_mode_clicked)

        self.media_player_main = QMediaPlayer(None, QMediaPlayer.VideoSurface)
        self.media_player_science = QMediaPlayer(None, QMediaPlayer.VideoSurface)
        self.media_player_main.setVideoOutput(self.camera_main_widget) 
        self.media_player_science.setVideoOutput(self.camera_science_widget) 
        
        #self.media_player_main.setMedia(QMediaContent(QUrl("rtsp://admin:admin@192.168.144.61:69/")))
        # self.media_player_science.setMedia(QMediaContent(QUrl("rtsp://admin:admin@192.168.144.61:69/")))
        #self.media_player_main.play()
        #self.media_player_science.play()

        #self.media_player = QMediaPlayer(None, QMediaPlayer.VideoSurface)
        #self.media_player.setVideoOutput(self.camera_main_widget)

        #media_content = QMediaContent(QUrl("http://commondatastorage.googleapis.com/gtv-videos-bucket/sample/ElephantsDream.mp4"))
        #self.media_player.setMedia(media_content)
        #self.media_player.play()

        #self.media_player_2 = QMediaPlayer(None, QMediaPlayer.VideoSurface)
        #self.media_player_2.setVideoOutput(self.camera_science_widget)

        #media_content = QMediaContent(QUrl("http://commondatastorage.googleapis.com/gtv-videos-bucket/sample/TearsOfSteel.mp4"))
        #self.media_player_2.setMedia(media_content)
        #self.media_player_2.play()
        

    def handle_service_unavailability(self, sender_rb, service_name):
        sender_rb.setAutoExclusive(False)
        sender_rb.setChecked(False)
        sender_rb.setAutoExclusive(True)
        self.ui_node.get_logger().warn('%s service not available.' % service_name)
        QMessageBox.warning(self, "Service Not Available", "The %s service is not available." % service_name)
    
    def drivetrain_arbitration_clicked(self):
        sender_rb = self.sender()
        if sender_rb is None:
            return

        dt_arbitration_client = self.ui_node.create_client(DriveTrainArbitration, '/rover/drive_train/set_arbitration')
        drivetrain_req = DriveTrainArbitration.Request()

        if not dt_arbitration_client.wait_for_service(timeout_sec=1.0):
            self.handle_service_unavailability(sender_rb, "drive_train_arbitration")
            return

        if sender_rb == self.rb_dt_none:
            drivetrain_req.target_arbitration.arbitration = DriveTrainArbitration.Constant.NONE
        elif sender_rb == self.rb_dt_teleop:
            drivetrain_req.target_arbitration.arbitration = DriveTrainArbitration.Constant.TELEOP
        elif sender_rb == self.rb_dt_autonomus:
            drivetrain_req.target_arbitration.arbitration = DriveTrainArbitration.Constant.AUTONOMUS

        response = dt_arbitration_client.call(drivetrain_req)
        self.ui_node.get_logger().info("Response : " + str(response.current_arbitration))

    def light_mode_clicked(self):
        sender_rb = self.sender()
        if sender_rb is None:
            return

        lights_client = self.ui_node.create_client(LightControl, '/rover/auxiliary/set/lights')
        lights_req = LightControl.Request()

        if not lights_client.wait_for_service(timeout_sec=1.0):
            self.handle_service_unavailability(sender_rb, "light_control")
            return

        if sender_rb == self.rb_normal_light:
            lights_req.index = LightControl.Request.LIGHT
            if sender_rb.isChecked():
               lights_req.enable = True
            else:
                lights_req.enable = False
        elif sender_rb == self.rb_infrared_light:
            lights_req.index = LightControl.Request.LIGHT_INFRARED
            if sender_rb.isChecked():
               lights_req.enable = True
            else:
                lights_req.enable = False

        response = lights_client.call(lights_req)
        self.ui_node.get_logger().info("Response : " + str(response.success))
        
    def antenna_arbitration_clicked(self):
        sender_rb = self.sender()
        if sender_rb is None:
            return

        antenna_client = self.ui_node.create_client(AntennaArbitration, '/base/antenna/set_arbitration')
        antenna_req = AntennaArbitration.Request()

        if not antenna_client.wait_for_service(timeout_sec=1.0):
            self.handle_service_unavailability(sender_rb, "antenna_arbitration")
            return

        if sender_rb == self.rb_ant_none:
            antenna_req.target_arbitration = AntennaArbitration.Request.NOT_MOVING
        elif sender_rb == self.rb_ant_teleop:
            antenna_req.target_arbitration = AntennaArbitration.Request.TELEOP
        elif sender_rb == self.rb_ant_autonomus:
            antenna_req.target_arbitration = AntennaArbitration.Request.AUTONOMUS

        response = antenna_client.call(antenna_req)
        self.ui_node.get_logger().info("Response : " + str(response.current_arbitration))

    def joydemux_clicked(self):
        sender_rb = self.sender()
        if sender_rb is None:
            return
        
        joydemux_client = self.ui_node.create_client(JoyDemuxSetState, '/joy/demux_control')
        joydemux_req = JoyDemuxSetState.Request()

        if not joydemux_client.wait_for_service(timeout_sec=1.0):
            sender_rb.setChecked(False)
            self.handle_service_unavailability(sender_rb, "joy_demux")
            return

        if sender_rb in (self.rb_joy_drivetrain_main, self.rb_joy_drivetrain_sec):
            joydemux_req.destination = JoyDemuxSetState.Request.DEST_DRIVE_TRAIN
        elif sender_rb in (self.rb_joy_antenna_main, self.rb_joy_antenna_sec):
            joydemux_req.destination = JoyDemuxSetState.Request.DEST_ANTENNA
        elif sender_rb in (self.rb_joy_arm_main, self.rb_joy_arm_sec):
            joydemux_req.destination = JoyDemuxSetState.Request.DEST_ARM
        elif sender_rb in (self.rb_joy_none_main, self.rb_joy_none_sec):
            joydemux_req.destination = JoyDemuxSetState.Request.DEST_NONE

        if sender_rb in (self.rb_joy_drivetrain_main, self.rb_joy_antenna_main,
                         self.rb_joy_arm_main, self.rb_joy_none_main):
            joydemux_req.controller_type = JoyDemuxSetState.Request.CONTROLLER_MAIN
        else:
            joydemux_req.controller_type = JoyDemuxSetState.Request.CONTROLLER_SECONDARY

        if self.rb_joy_force.isChecked():
            joydemux_req.force = True

        response = joydemux_client.call(joydemux_req)
        self.ui_node.get_logger().info("Response : " + str(response.success))

    def __del__(self):
        self.ui_node.destroy_subscription(self.joydemux_sub)

    def update_joydemux_button_status(self, msg):
        if msg.controller_main_topic == JoyDemuxStatus.DEST_DRIVE_TRAIN:
            self.rb_joy_drivetrain_main.setChecked(True)
        elif msg.controller_main_topic == JoyDemuxStatus.DEST_ARM:
            self.rb_joy_arm_main.setChecked(True)
        elif msg.controller_main_topic == JoyDemuxStatus.DEST_ANTENNA:
            self.rb_joy_antenna_main.setChecked(True)
        elif msg.controller_main_topic == JoyDemuxStatus.DEST_NONE:
            self.rb_joy_none_main.setChecked(True)

        if msg.controller_secondary_topic == JoyDemuxStatus.DEST_DRIVE_TRAIN:
            self.rb_joy_drivetrain_sec.setChecked(True)
        elif msg.controller_secondary_topic == JoyDemuxStatus.DEST_ARM:
            self.rb_joy_arm_sec.setChecked(True)
        elif msg.controller_secondary_topic == JoyDemuxStatus.DEST_ANTENNA:
            self.rb_joy_antenna_sec.setChecked(True)
        elif msg.controller_secondary_topic == JoyDemuxStatus.DEST_NONE:
            self.rb_joy_none_sec.setChecked(True)
