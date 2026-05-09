#ifndef QUTILITY_BAR_BOTTOM_HPP
#define QUTILITY_BAR_BOTTOM_HPP

#include "UI_UtilityBarBottom.h"
#include <rclcpp/rclcpp.hpp>
#include <rover_msgs/msg/arm_msg.hpp>

class QUtilityBarBottom : public QWidget
{
    Q_OBJECT

    static constexpr const char* ARM_STATUS_TOPIC = "/rover/arm/joints_status";
    static constexpr const float MAX_TORQUE_GRIPPER = 0.30F;

  public:
    explicit QUtilityBarBottom(std::shared_ptr<rclcpp::Node> node_, QWidget* parent_);

  signals:
    void seeHistory();
    void updateTorqueIndicator(float torque_);

  private slots:
    void updateIndicator(const float torque_);

  private:
    void CB_armStatus(const rover_msgs::msg::ArmMsg& msg_);

    Ui::UtilityBarBottom _ui;
    std::shared_ptr<rclcpp::Node> _node;

    std::shared_ptr<rclcpp::Subscription<rover_msgs::msg::ArmMsg>> _sub_armJointStatus;

    float _lastColor;
};

#endif  // QUTILITY_BAR_BOTTOM_HPP
