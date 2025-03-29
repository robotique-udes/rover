#ifndef __QARUCO_HPP__
#define __QARUCO_HPP__

// ROS
#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/gps.hpp"

// QT
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QWidget>
#include "UI_Aruco.h"

class QAruco : public QWidget
{
    Q_OBJECT

  public:
    QAruco(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_);

  private:
    //void gpsCallback(const rover_msgs::msg::Gps::SharedPtr rosMsg_);

    //rclcpp::Subscription<rover_msgs::msg::Gps>::SharedPtr _sub_gps;
    void startDetection(std::string camURL_);

    std::string _camURL;
    std::shared_ptr<rclcpp::Node> _node;
    Ui::Aruco _ui;
};

#endif  // __QEXAMPLE_HPP__
