#ifndef QVIDEO_PLAYER_HPP
#define QVIDEO_PLAYER_HPP

#include "QVideoPlayerWidget.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/aruco.hpp"

#include <QtWidgets/QGridLayout>
#include <QtWidgets/QWidget>

class QVideoManagerWidget : public QWidget
{
    Q_OBJECT

    static constexpr uint16_t DELAY_DETECTION_MANAGER_UPDATE = 500U;
    static constexpr uint16_t NBR_CAM_TO_TRACK = 6U;
    static constexpr const char* CAM_MAIN = "rtsp://192.168.144.30";
    static constexpr const char* CAM_ANTENNA = "rtsp://192.168.144.31";
    static constexpr const char* CAM_ODOM = "rtsp://192.168.144.32";
    static constexpr const char* CAM_GRIPPER_1 = "rtsp://192.168.144.35";
    static constexpr const char* CAM_GRIPPER_2 = "rtsp://192.168.144.36";
    static constexpr const char* CAM_6 = "";

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

    std::shared_ptr<rclcpp::Node> _node;

    QGridLayout _videoPlayerLayout;

    std::shared_ptr<QPlayerWorker> _playerWorkerThread;

    std::shared_ptr<rclcpp::Client<rover_msgs::srv::ArucoDetection>> _client_arucoDetectionManager;
    std::shared_ptr<rclcpp::Subscription<rover_msgs::msg::Aruco>> _sub_arucoDetection;
    rclcpp::TimerBase::SharedPtr _timer_detectionManagerUpdate;

    std::array<std::shared_ptr<QVideoPlayerWidget>, NBR_CAM_TO_TRACK> _videoPlaysWidgets;
    std::array<const char*, 6> _cameras_urls = {CAM_MAIN, CAM_ANTENNA, CAM_ODOM, CAM_GRIPPER_1, CAM_GRIPPER_2, CAM_6};
};

#endif  // QVIDEO_PLAYER_HPP