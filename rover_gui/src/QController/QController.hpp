#ifndef QCONTROLLER_QCONTROLLER_HPP
#define QCONTROLLER_QCONTROLLER_HPP


// ROS
#include <rclcpp/rclcpp.hpp>

// QT
#include "ui_Controller.h"
#include <QtWidgets/QGridLayout>


class QController : public QWidget
{
    Q_OBJECT

  public:
    QController(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_);

  private:
    Ui::Controller _ui;
};

#endif  // QCONTROLLER_QCONTROLLER_HPP