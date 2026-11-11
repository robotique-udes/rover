#include "QJoint.hpp"
#include <rover_lib2/helpers/assert.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <ui_Joint.h>

QJoint::QJoint(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_):
    QWidget(parent_),
    _node(std::move(guiNode_))
{
    ASSERT_COND(_node != nullptr);

    _ui.setupUi(this);

    connect(this, &QJoint::jointCommandReceived, this, &QJoint::JointCommandChanged);

    rclcpp::QoS jointCommandQoS(rclcpp::KeepLast(1));
    jointCommandQoS.transient_local();

    _subControlMode = _node->create_subscription<rover_msgs::msg::JointInfo>(
        "/rover/arm/mode",
        jointCommandQoS,
    [this](const rover_msgs::msg::JointInfo::SharedPtr msg_)
    {
        emit jointCommandReceived(msg_->data);
    });
}
    void QJoint::jointCommandChanged(uint8_t mode)
    {
        if (mode==0)
        {_ui.armModeLabel->setText("Control Mode: JOINT");}
        else if (mode==1)
        {_ui.armModeLabel->setText("Control Mode: CARTESIAN");}
    }
