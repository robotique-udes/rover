#ifndef QJOINT_QJOINT_HPP
#define QJOINT_QJOINT_HPP

#include <rclcpp/rclcpp.hpp>
#include <QWidget>
#include "ui_Joint.h"

class QJoint : public QWidget
{
    Q_OBJECT

public:
    QJoint(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_ = nullptr);

  private:
    std::shared_ptr<rclcpp::Node> _node;
    Ui::Joint _ui;
};

#endif // QJOINT_QJOINT_HPP