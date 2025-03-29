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

  public:
    QAruco(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_);

  private slots:
    void onDetectionStarted(bool success);

  

  private:
    void startDetection();

    QPlayerWorker _playerWorkerThread;


    std::string _camURL;
    std::shared_ptr<rclcpp::Node> _node;
    Ui::VideoPlayer _ui;

    std::shared_ptr<rclcpp::Client<rover_msgs::srv::ArucoDetection>> _client_ArucoDetectionManager;

};

#endif  // __QEXAMPLE_HPP__
