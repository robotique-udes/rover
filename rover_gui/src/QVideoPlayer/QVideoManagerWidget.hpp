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
#include "rover_lib2/helpers/cameraInterface.hpp"

#include <rover_msgs/srv/panorama.hpp>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QWidget>

enum class eTabIndex : uint8_t
{
    GRID = 0,
    ALT,
    ARM3,
    ARM4
};

class QVideoManagerWidget : public QWidget
{
    Q_OBJECT

    static constexpr const char* SERVICE_ARUCO_NAME = "/rover/cameras/aruco_detection_management";
    static constexpr const char* TOPIC_ARUCO_DETECTIONS = "/rover/cameras/aruco_detected";
    static constexpr const char* SERVICE_RECORDING_NAME = "/rover/cameras/media_server_control";
    static constexpr const char* TOPIC_RECORDING_INFO = "/rover/camera/recordings_info";
    static constexpr const char* CAMERA_PTZ_CMD_TOPIC_GUI = "/rover/camera/PTZ_cmd/GUI";
    static constexpr const char* CAMERA_PTZ_CONFIG_TOPIC_GUI = "/rover/camera/PTZ_config/GUI";
    static constexpr const char* CAMERA_POWER_TOPIC_GUI = "/rover/camera/power_cmd/GUI";
    static constexpr const char* CAMERA_STATUS_TOPIC = "/rover/camera/PTZ_status";
    static constexpr const char* SERVICE_PANORAMA_NAME = "/rover/video/panorama";

    static constexpr uint16_t DELAY_DETECTION_MANAGER_UPDATE = 5000U;
    static constexpr uint16_t TIMEOUT_SERVICE_AVAILABLE = 50U;
    static constexpr uint16_t NBR_CAM_TO_TRACK = 6U;
    static constexpr uint16_t CAMERA_CENTER_ANGLE = 180U;

    static constexpr float ALT_CAM_LAYOUT_PROPORTION = 0.7F;

    static constexpr std::array<const char*, 5> CAMERA_NAME_ORDER = {
        "Main",
        "Antenna",
        "Front-Side",
        "Arm-Top",
        "Arm-Side",
    };

    static constexpr float CAMERA_MAX_SPEED = 10.0F;
    static constexpr float CAMERA_MIN_ANGLE = 0.0F;
    static constexpr float CAMERA_MAX_ANGLE = 360.0F;

  public:
    QVideoManagerWidget(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_);
    ~QVideoManagerWidget();

    void CB_updateArucoDetectionManager(void);
    void CB_displayArucoDetected(rover_msgs::msg::Aruco msg_);
  private slots:
    void onArucoDetectionIsLive(std::vector<std::string> liveUrlList_);
    void onSetCursorWaiting(bool waiting_);
    
    /**
     * @brief set the PTZ cmd
     * 
     * @param yaw_ angle in degrees
     * @param id_ camera id
     */
    void setPTZCmd(float yaw_, size_t id_);
    void onTabChanged(uint16_t index_);

  private:
    void initWidget(void);
    void initArucoPublisher(void);
    void initArucoClient(void);

    void clearLayout(QLayout* layout_);

    void initCameraControlClient(void);
    void initCameraAnglePublisher(void);
    void initCameraListSubscriber(void);
    void initCameraStatusSubscriber(void);

    void initPanoramaClient(void);
    void initCameraInterface(void);

    void setSplitterInitialGeometry(void);

    std::shared_ptr<rclcpp::Node> _node;

    CameraInterface _cameraInterface;

    std::shared_ptr<QPlayerWorker> _playerWorkerThreadAruco;
    std::array<std::shared_ptr<QRecordingWorker>, NBR_CAM_TO_TRACK> _playerWorkerThreadRecording;
    std::shared_ptr<QPanoramaWorker> _panoramaWorkerThread;

    QTabWidget _tabWidget;
    QVBoxLayout _mainLayout;

    QGridLayout _gridLayout;
    QWidget _gridContainer;

    QHBoxLayout _altLayout;
    QVBoxLayout _vSubLayout;
    QVBoxLayout _arm3Layout;
    QVBoxLayout _arm3SubLayout;
    QGridLayout _arm4Layout;
    QWidget _vSubLayoutContainer;
    QWidget _altLayoutContainer;
    QWidget _arm3LayoutContainer;
    QWidget _arm3SubLayoutContainer;
    QWidget _arm4LayoutContainer;

    QSplitter _splitter;
    QSplitter _arm3Splitter;
    QPushButton _resetLayout_PB;

    std::shared_ptr<rclcpp::Client<rover_msgs::srv::ArucoDetection>> _client_arucoDetectionManager;
    std::shared_ptr<rclcpp::Subscription<rover_msgs::msg::Aruco>> _sub_arucoDetection;
    rclcpp::TimerBase::SharedPtr _timer_detectionManagerUpdate;

    std::shared_ptr<rclcpp::Client<rover_msgs::srv::CameraControl>> _client_cameraControlManager;
    std::shared_ptr<rclcpp::Subscription<rover_msgs::msg::CameraList>> _sub_cameraList;
    std::shared_ptr<rclcpp::Subscription<rover_msgs::msg::CameraControl>> _sub_cameraStatus;
    rclcpp::TimerBase::SharedPtr _timer_clientCameraControlHealth;

    std::shared_ptr<rclcpp::Client<rover_msgs::srv::Panorama>> _client_panoramique;

    std::array<std::unique_ptr<QVideoPlayerWidget>, NBR_CAM_TO_TRACK> _videoPlaysWidgets;
};

#endif
