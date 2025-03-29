#ifndef __QARUCO_HPP__
#define __QARUCO_HPP__

// ROS
#include "rclcpp/rclcpp.hpp"

// QT
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QWidget>
#include "UI_VideoPlayer.h"
#include "Worker/QPlayerWorker.hpp"

class QAruco : public QWidget
{
    Q_OBJECT

    static constexpr u_int16_t DELAY_DETECTION_MANAGER_UPDATE= 5000U;


  public:
    QAruco(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_);
    
    void startDetection();
    void stopDetection();
    void handleDetection();

    void CB_updateDetectionManager();

  private slots:
    void onDetectionHandledSuccessfully(bool success);

  private slots:
    void onUrlFoundInDetection(bool was_found_);

  

  private:



    QPlayerWorker _playerWorkerThread;


    std::string _camURL = "rtsp://127.0.0.1:8554/live";
    std::shared_ptr<rclcpp::Node> _node;
    Ui::VideoPlayer _ui;

    std::shared_ptr<rclcpp::Client<rover_msgs::srv::ArucoDetection>> _client_ArucoDetectionManager;
    rclcpp::TimerBase::SharedPtr _timer_detectionManagerUpdate;


};

#endif  // __QEXAMPLE_HPP__
