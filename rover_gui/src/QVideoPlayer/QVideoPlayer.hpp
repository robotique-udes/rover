#ifndef __QVIDEO_PLAYER_HPP__
#define __QVIDEO_PLAYER_HPP__

#include "rclcpp/rclcpp.hpp"
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QWidget>

#include "QVideoPlayerWidget.hpp"

class QVideoPlayer : public QWidget
{
    Q_OBJECT
    static constexpr u_int16_t DELAY_DETECTION_MANAGER_UPDATE= 5000U;


  public:
    QVideoPlayer(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_);
    void CB_updateDetectionManager();

    
  private slots:
      void onUrlFoundInDetection(std::vector<std::string> live_url_list_);

  private:
    std::shared_ptr<rclcpp::Node> _node;

    QGridLayout _dashboardLayout;  // CHANGER !!
    QVideoPlayerWidget _videoFrameWidget;

    std::shared_ptr<rclcpp::Client<rover_msgs::srv::ArucoDetection>> _client_arucoDetectionManager;
    rclcpp::TimerBase::SharedPtr _timer_detectionManagerUpdate;

};
#warning green_checked needs to only apply to video player!!

#endif  //__QVIDEO_PLAYER_HPP__