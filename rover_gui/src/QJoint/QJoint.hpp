#ifndef QJOINT_QJOINT_HPP
#define QJOINT_QJOINT_HPP

#include <rclcpp/rclcpp.hpp>
#include <QWidget>
#include <ui_Joint.h>
#include <rover_msgs/msg/joint_info.hpp>

class QJoint : public QWidget
{
    Q_OBJECT

public:
    QJoint(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_ = nullptr);

signals:
    void jointCommandReceived(uint8_t armMode);


private slots:
    void jointCommandChanged(uint8_t mode);
private:
    std::shared_ptr<rclcpp::Node> _node;
    Ui::Joint _ui;
    rclcpp::Subscription<rover_msgs::msg::JointInfo>::SharedPtr _subControlMode;
};

#endif // QJOINT_QJOINT_HPP