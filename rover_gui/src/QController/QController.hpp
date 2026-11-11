#ifndef QCONTROLLER_HPP
#define QCONTROLLER_HPP

// ROS
#include <rclcpp/rclcpp.hpp>
#include <rover_msgs/msg/joy.hpp>

// QT
#include "ui_Controller.h"
#include <QtWidgets/QWidget>
#include <vector>

class QController : public QWidget
{
    Q_OBJECT

  public:
    QController(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_ = nullptr);

  private:
    std::shared_ptr<rclcpp::Node> _node;
    Ui::Controller _ui;
    rclcpp::Subscription<rover_msgs::msg::Joy>::SharedPtr _joySub;

  signals:
    void joyInputReceived(const std::vector<float>& joyData);
};

#endif  // QCONTROLLER_HPP