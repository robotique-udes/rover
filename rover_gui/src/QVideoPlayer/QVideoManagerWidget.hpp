#ifndef QVIDEO_PLAYER_HPP
#define QVIDEO_PLAYER_HPP

#include "QVideoPlayerWidget.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/aruco.hpp"
#include "rover_msgs/msg/camera_list.hpp"
#include "rover_lib2/helpers/constants.hpp"

#include <QtWidgets/QGridLayout>
#include <QtWidgets/QWidget>

class QVideoManagerWidget : public QWidget
{
    Q_OBJECT

    static constexpr const char* SERVICE_ARUCO_NAME = "/rover/cameras/aruco_detection_management";
    static constexpr const char* TOPIC_ARUCO_DETECTIONS = "/rover/cameras/aruco_detected";
    static constexpr const char* SERVICE_RECORDING_NAME = "/rover/cameras/media_server_control";
    static constexpr const char* TOPIC_RECORDING_INFO = "rover/camera/recordings_info";

    static constexpr uint16_t DELAY_DETECTION_MANAGER_UPDATE = 500U;
    static constexpr uint16_t NBR_CAM_TO_TRACK = 6U;
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

  private:
    void initWidget(void);
    void initArucoPublisher(void);
    void initArucoClient(void);

    void initCameraControlClient(void);

    void initCameraControlPublisher(void);

    std::shared_ptr<rclcpp::Node> _node;

    QGridLayout _videoPlayerLayout;

    std::shared_ptr<QPlayerWorker> _playerWorkerThread;
    std::shared_ptr<QPlayerWorker> _playerWorkerThread2;

    std::shared_ptr<rclcpp::Client<rover_msgs::srv::ArucoDetection>> _client_arucoDetectionManager;
    std::shared_ptr<rclcpp::Subscription<rover_msgs::msg::Aruco>> _sub_arucoDetection;
    rclcpp::TimerBase::SharedPtr _timer_detectionManagerUpdate;

    std::shared_ptr<rclcpp::Client<rover_msgs::srv::CameraControl>> _client_cameraControlManager;
    std::shared_ptr<rclcpp::Subscription<rover_msgs::msg::CameraList>> _sub_cameraList;

    std::array<std::unique_ptr<QVideoPlayerWidget>, NBR_CAM_TO_TRACK> _videoPlaysWidgets;
};

#endif  // QVIDEO_PLAYER_HPP
