#ifndef QVIDEO_PLAYER_HPP
#define QVIDEO_PLAYER_HPP

#include "QVideoPlayerWidget.hpp"

#include <QBoxLayout>
#include <QTabWidget>
#include <QSplitter>
#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/aruco.hpp"
#include "rover_msgs/msg/camera_list.hpp"
#include "rover_msgs/msg/camera_control.hpp"
#include "rover_lib2/helpers/constants.hpp"

#include <QtWidgets/QGridLayout>
#include <QtWidgets/QWidget>

class QVideoManagerWidget : public QWidget
{
    Q_OBJECT

    static constexpr const char* SERVICE_ARUCO_NAME = "/rover/cameras/aruco_detection_management";
    static constexpr const char* TOPIC_ARUCO_DETECTIONS = "/rover/cameras/aruco_detected";
    static constexpr const char* SERVICE_RECORDING_NAME = "/rover/cameras/media_server_control";
    static constexpr const char* TOPIC_RECORDING_INFO = "/rover/camera/recordings_info";
    static constexpr const char* CAMERA_ANGLE_CONTROL_TOPIC = "/rover/cameras/pos_control";

    static constexpr uint16_t DELAY_DETECTION_MANAGER_UPDATE = 5000U;
    static constexpr uint16_t TIMEOUT_SERVICE_AVAILABLE = 50U;
    static constexpr uint16_t NBR_CAM_TO_TRACK = 6U;

    static constexpr float ALT_CAM_LAYOUT_PROPORTION = 0.7f;

    static constexpr std::array<const char*, 5> CAMERA_NAME_ORDER = {
        "Main",
        "Antenna",
        "Front-Side",
        "Arm-Top",
        "Arm-Side",
    };

  public:
    QVideoManagerWidget(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_);

    void CB_updateArucoDetectionManager(void);
    void CB_displayArucoDetected(rover_msgs::msg::Aruco msg_);

  private slots:
    void onArucoDetectionIsLive(std::vector<std::string> liveUrlList_);
    void onSetCursorWaiting(bool waiting_);
    void CB_pubCameraAngle(std::string camURL_, float pitch_);
    void onTabChanged(uint16_t index_);

  private:
    void initWidget(void);
    void initArucoPublisher(void);
    void initArucoClient(void);

    void clearLayout(QLayout* layout_);

    void initCameraControlClient(void);
    void initCameraAnglePublisher(void);
    void initCameraControlSubscriber(void);

    void setSplitterInitialGeometry(void);

    std::shared_ptr<rclcpp::Node> _node;

    std::shared_ptr<QPlayerWorker> _playerWorkerThreadAruco;
    std::array<std::shared_ptr<QRecordingWorker>, NBR_CAM_TO_TRACK> _playerWorkerThreadRecording;

    QTabWidget _tabWidget;
    QVBoxLayout _mainLayout;

    QGridLayout _gridLayout;
    QWidget _gridContainer;

    QHBoxLayout _altLayout;
    QVBoxLayout _vSubLayout;
    QWidget _vSubLayoutContainer;
    QWidget _altLayoutContainer;

    QSplitter _splitter;
    QPushButton _resetLayout_PB;

    std::shared_ptr<rclcpp::Client<rover_msgs::srv::ArucoDetection>> _client_arucoDetectionManager;
    std::shared_ptr<rclcpp::Subscription<rover_msgs::msg::Aruco>> _sub_arucoDetection;
    rclcpp::TimerBase::SharedPtr _timer_detectionManagerUpdate;

    std::shared_ptr<rclcpp::Client<rover_msgs::srv::CameraControl>> _client_cameraControlManager;
    std::shared_ptr<rclcpp::Subscription<rover_msgs::msg::CameraList>> _sub_cameraList;
    std::shared_ptr<rclcpp::Publisher<rover_msgs::msg::CameraControl>> _pub_cameraAngle;
    rclcpp::TimerBase::SharedPtr _timer_clientCameraControlHealth;
    rclcpp::TimerBase::SharedPtr _timer_pubCameraAngle;

    std::array<std::unique_ptr<QVideoPlayerWidget>, NBR_CAM_TO_TRACK> _videoPlaysWidgets;
};

#endif
