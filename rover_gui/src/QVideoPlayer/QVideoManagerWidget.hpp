#ifndef __QVIDEO_PLAYER_HPP__
#define __QVIDEO_PLAYER_HPP__

#include "rclcpp/rclcpp.hpp"
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QWidget>

#include "QVideoPlayerWidget.hpp"

class QVideoManagerWidget : public QWidget
{
    Q_OBJECT
    static constexpr uint16_t DELAY_DETECTION_MANAGER_UPDATE = 5000U;
    static constexpr uint16_t NBR_CAM_TO_TRACK = 6U;

  public:
    QVideoManagerWidget(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_);
    void CB_updateArucoDetectionManager(void);

  private slots:
    void onArucoDetectionIsLive(std::vector<std::string> live_url_list_);

  private:

    std::shared_ptr<rclcpp::Node> _node;

    QGridLayout _dashboardLayout;  // CHANGER !!

    std::shared_ptr<QPlayerWorker> _playerWorkerThread;

    std::shared_ptr<rclcpp::Client<rover_msgs::srv::ArucoDetection>> _client_arucoDetectionManager;
    rclcpp::TimerBase::SharedPtr _timer_detectionManagerUpdate;

    std::array<std::shared_ptr<QVideoPlayerWidget>, NBR_CAM_TO_TRACK> _videoPlaysWidgets;
    std::array<std::string, 6> _cameras_urls = {"rtsp://127.0.0.1:8554/live","1","2","3","4","5"};
};
#warning green_checked needs to only apply to video player!!

#endif  //__QVIDEO_PLAYER_HPP__