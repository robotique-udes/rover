#ifndef QVIDEO_PLAYER_HPP
#define QVIDEO_PLAYER_HPP

#include "QVideoPlayerWidget.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/aruco.hpp"
#include "rover_lib2/helpers/constants.hpp"

#include <QtWidgets/QGridLayout>
#include <QtWidgets/QWidget>
#include <qboxlayout.h>
#include <qtabwidget.h>

class QVideoManagerWidget : public QWidget
{
    Q_OBJECT

    static constexpr const char* SERVICE_ARUCO_NAME = "/rover/cameras/aruco_detection_management";
    static constexpr const char* TOPIC_ARUCO_DETECTIONS = "/rover/cameras/aruco_detected";

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
    void onTabChanged(int index);

  private:
    void initWidget(void);
    void initArucoPublisher(void);
    void initArucoClient(void);

    void clearLayout(QLayout* layout_);

    std::shared_ptr<rclcpp::Node> _node;

    QTabWidget _tabWidget;

    QVBoxLayout _mainLayout;

    QGridLayout _gridLayout;
    QWidget _gridContainer;

    QHBoxLayout _altLayout;
    QVBoxLayout _vLayout;
    QWidget _vLayoutContainer;
    QWidget _altLayoutContainer;

    std::shared_ptr<QPlayerWorker> _playerWorkerThread;

    std::shared_ptr<rclcpp::Client<rover_msgs::srv::ArucoDetection>> _client_arucoDetectionManager;
    std::shared_ptr<rclcpp::Subscription<rover_msgs::msg::Aruco>> _sub_arucoDetection;
    rclcpp::TimerBase::SharedPtr _timer_detectionManagerUpdate;

    std::array<std::unique_ptr<QVideoPlayerWidget>, NBR_CAM_TO_TRACK> _videoPlaysWidgets;
};

#endif
