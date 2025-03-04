#ifndef __QEXAMPLE_HPP__
#define __QEXAMPLE_HPP__

// ROS
#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/gps.hpp"

// QT
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QWidget>
#include "UI_Example.h"

class QExample : public QWidget
{
    Q_OBJECT

  public:
    QExample(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_);

  private:
    void gpsCallback(const rover_msgs::msg::Gps::SharedPtr rosMsg_);

    rclcpp::Subscription<rover_msgs::msg::Gps>::SharedPtr _sub_gps;
    std::shared_ptr<rclcpp::Node> _node;
    Ui::Example _ui;
};

#endif  // __QEXAMPLE_HPP__

/*
  Use this command to publish on the /rover/gps/position topic :
  ros2 topic pub /rover/gps/position rover_msgs/msg/Gps '{latitude: 121.2, longitude: -13.41, height: 0.0, heading_gps: 0,
  heading_track: 0, speed: 0, satellite: 8, heading: 0}'
*/
