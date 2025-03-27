#ifndef __QVIDEOPLAYER_HPP__
#define __QVIDEOPLAYER_HPP__

// ROS
#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/gps.hpp"

// QT
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QWidget>
#include "UI_VideoPlayer.h"

class QVideoPlayer : public QWidget
{
    Q_OBJECT

  public:
    QVideoPlayer(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_);

  private:
    //void gpsCallback(const rover_msgs::msg::Gps::SharedPtr rosMsg_);

    //rclcpp::Subscription<rover_msgs::msg::Gps>::SharedPtr _sub_gps;
    std::shared_ptr<rclcpp::Node> _node;
    Ui::VideoPlayer _ui;
};

#endif  // __QEXAMPLE_HPP__
