#ifndef QVIDEO_PLAYER_HPP
#define QVIDEO_PLAYER_HPP

#include "QVideoPlayerWidget.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/aruco.hpp"
#include "rovus_lib/camera_info.hpp"

#include <QtWidgets/QGridLayout>
#include <QtWidgets/QWidget>

class QVideoManagerWidget : public QWidget
{
    Q_OBJECT

    static constexpr uint16_t DELAY_DETECTION_MANAGER_UPDATE = 500U;
    static constexpr uint16_t NBR_CAM_TO_TRACK = 6U;
    static constexpr const char* CAM_1_NAME = "Main";
    static constexpr const char* CAM_2_NAME = "Antenna";
    static constexpr const char* CAM_3_NAME = "Odometry";
    static constexpr const char* CAM_4_NAME = "Gripper1";
    static constexpr const char* CAM_5_NAME = "Gripper2";
    static constexpr const char* CAM_6_NAME = "";

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

    /**
     * @brief Initialise the CameraControl client and passes it to each widget
     *
     */
    void initCameraControlClient(void);

    std::shared_ptr<rclcpp::Node> _node;

    QGridLayout _videoPlayerLayout;

    std::shared_ptr<QPlayerWorker> _playerWorkerThread;

    std::shared_ptr<rclcpp::Client<rover_msgs::srv::ArucoDetection>> _client_arucoDetectionManager;
    std::shared_ptr<rclcpp::Subscription<rover_msgs::msg::Aruco>> _sub_arucoDetection;
    rclcpp::TimerBase::SharedPtr _timer_detectionManagerUpdate;

    std::shared_ptr<rclcpp::Client<rover_msgs::srv::CameraControl>> _client_cameraControlManager;

    std::array<std::shared_ptr<QVideoPlayerWidget>, NBR_CAM_TO_TRACK> _videoPlaysWidgets;
    std::array<std::string, 6> _cameras_urls = {CameraInfo::CameraIP.at(CAM_1_NAME),
                                                CameraInfo::CameraIP.at(CAM_2_NAME),
                                                CameraInfo::CameraIP.at(CAM_3_NAME),
                                                CameraInfo::CameraIP.at(CAM_4_NAME),
                                                CameraInfo::CameraIP.at(CAM_5_NAME),
                                                ""};
};

#endif  // QVIDEO_PLAYER_HPP