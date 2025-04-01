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
    QVideoPlayerWidget(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_);
    
    void startDetection();
    void stopDetection();
    void handleDetection();
    void arucoStillAliveUpdate();
    std::string getCamURL();
    void setArucoClientManager(std::shared_ptr<rclcpp::Client<rover_msgs::srv::ArucoDetection>> client_);
    QPlayerWorker* getWorker(void);
    void setURLFound(bool urlFound_);


  private slots:
    void onDetectionHandledSuccessfully(bool success);

  

  private:

    std::string _camURL = "rtsp://127.0.0.1:8554/live";
    std::shared_ptr<rclcpp::Node> _node;
    Ui::VideoPlayer _ui;

    std::shared_ptr<rclcpp::Client<rover_msgs::srv::ArucoDetection>> _client_arucoManager = nullptr;

    QPlayerWorker _playerWorkerThread;

    bool _urlFound = false;




};

#endif  // __QVIDEOPLAYERWIDGER_HPP___
