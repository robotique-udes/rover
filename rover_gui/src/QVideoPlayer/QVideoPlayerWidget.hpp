#ifndef __QVIDEOPLAYERWIDGER_HPP__
#define __QVIDEOPLAYERWIDGER_HPP__

// ROS
#include "rclcpp/rclcpp.hpp"

// QT
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QWidget>
#include "UI_VideoPlayer.h"
#include "Worker/QPlayerWorker.hpp"

class QVideoPlayerWidget : public QWidget
{
    Q_OBJECT

    static constexpr uint64_t MAX_DELAY_SERVICE_CALL = 2000UL;

  public:
    QVideoPlayerWidget(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_,std::string url_,uint16_t tag_, std::shared_ptr<QPlayerWorker> worker_);

    void startDetection();
    void stopDetection();
    void handleArucoDetection();

    void handlePlayPauseButton();


    void arucoStillAliveUpdate(bool urlFound_);
    
    void setArucoClientManager(std::shared_ptr<rclcpp::Client<rover_msgs::srv::ArucoDetection>> client_);
    std::string getCamURL();


  private slots:
    void onDetectionHandledSuccessfully(bool success_,uint16_t tag_);
  private slots:
    void onArucoServerInfoFailed(bool success_);

  private:
    std::shared_ptr<rclcpp::Node> _node;
    Ui::VideoPlayer _ui;

    std::string _camURL = "";
    uint16_t _tag;

    std::shared_ptr<rclcpp::Client<rover_msgs::srv::ArucoDetection>> _client_arucoManager = nullptr;

    std::shared_ptr<QPlayerWorker> _playerWorkerThread;


};

#endif  // __QVIDEOPLAYERWIDGER_HPP___
