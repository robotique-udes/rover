#include "QUtilityBarBottom.hpp"

#include <algorithm>
#include <cmath>

QUtilityBarBottom::QUtilityBarBottom(std::shared_ptr<rclcpp::Node> node_, QWidget* parent_):
    QWidget(parent_),
    _node(node_)
{
    _ui.setupUi(this);
    _ui.gripperLabel->setText("Gripper: ");

    _ui.gripperIcon->setDisabled(true);
    _ui.gripperIcon->setFixedSize(18, 18);
    _ui.gripperIcon->setStyleSheet("background-color: rgb(0, 255, 0); border: 1px solid #202020;");

    _sub_armJointStatus = this->_node->create_subscription<rover_msgs::msg::ArmMsg>(ARM_STATUS_TOPIC,
                                                                                    1,
                                                                                    [this](const rover_msgs::msg::ArmMsg& msg_)
                                                                                    {
                                                                                        this->CB_armStatus(msg_);
                                                                                    });

    connect(_ui.notificationHistory_PB,
            &QPushButton::clicked,
            this,
            [this]()
            {
                emit seeHistory();
            });

    connect(this, &QUtilityBarBottom::updateTorqueIndicator, this, &QUtilityBarBottom::updateIndicator, Qt::QueuedConnection);

    this->raise();
}

void QUtilityBarBottom::CB_armStatus(const rover_msgs::msg::ArmMsg& msg_)
{
    emit this->updateTorqueIndicator(msg_.current_torque[rover_msgs::msg::ArmMsg::GRIPPER_CLOSE]);
}

void QUtilityBarBottom::updateIndicator(const float torque_)
{
    const float ratio = std::clamp(std::abs(torque_) / MAX_TORQUE_GRIPPER, 0.0F, 1.0F);
    const int red = static_cast<int>(255.0F * ratio);
    const int green = static_cast<int>(255.0F * (1.0F - ratio));

    _ui.gripperIcon->setStyleSheet(QString("background-color: rgb(%1, %2, 0); border: 1px solid #202020;").arg(red).arg(green));
}
