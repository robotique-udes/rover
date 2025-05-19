#include "arm_joint.hpp"
#include <rover_msgs/msg/detail/arm_msg__struct.hpp>
#include <rover_lib2/helpers/constants.hpp>

ArmJoint::ArmJoint(RoverCan2::Constant::eDeviceId deviceId_,
                   uint8_t rosArmSpeedMsgId_,
                   std::shared_ptr<CanMaster::SharedRosMsg<rover_msgs::msg::ArmMsg>> rosSharedMsg_):
    DeviceT(deviceId_,
            RoverCan2::Publisher<RoverCan2::Msgs::ArmSpeedCmd>(),
            RoverCan2::SubscriberMember(*this, &ArmJoint::CB_CAN_armPostitionStatus)),
    _rosArmSpeedMsgId(rosArmSpeedMsgId_),
    _rosSharedMsg(rosSharedMsg_)
{
    ASSERT_COND_MSG(_rosSharedMsg, "rosSharedMsg_ can't be nullptr");
}

void ArmJoint::rosElementInit(void)
{
    _pub_ArmPositionStatus = this->getAttachedNode()->create_publisher<rover_msgs::msg::ArmMsg>(ARM_POSITION_STATUS_TOPIC, 1);
    _rosSharedMsg->attachNewPub(this->getAttachedNode(), _pub_ArmPositionStatus, ARM_POSITION_STATUS_PUBLISH_FREQUENCY_HZ);

    _timerCanSend = this->getAttachedNode()->create_wall_timer(std::chrono::milliseconds(CAN_PUBLISH_PERIOD_MS),
                                                               [this]()
                                                               {
                                                                   this->CB_ROS_canSend();
                                                               });

    _sub_ArmPositionStatus
        = this->getAttachedNode()->create_subscription<rover_msgs::msg::ArmMsg>(ARM_CMD_TOPIC,
                                                                                QOS_DEFAULT,
                                                                                [this](const rover_msgs::msg::ArmMsg& rosMsg_)
                                                                                {
                                                                                    this->CB_ROS_armSpeedCmd(rosMsg_);
                                                                                });
}

void ArmJoint::rosElementClean(void)
{
    if (_pub_ArmPositionStatus)
    {
        _rosSharedMsg->removePub(_pub_ArmPositionStatus);
        _pub_ArmPositionStatus.reset();
    }
    if (_timerCanSend)
    {
        _timerCanSend.reset();
    }
    if (_sub_ArmPositionStatus)
    {
        _sub_ArmPositionStatus.reset();
    }
}

std::vector<RoverCan2::Constant::eDeviceId> ArmJoint::getManagedDevicesIds(void)
{
    return {this->getCanId()};
}

void ArmJoint::CB_CAN_armPostitionStatus(const RoverCan2::Msgs::ArmPositionStatus& msg_)
{
    _rosSharedMsg->get().getThreadSafeAccess().current_position[_rosArmSpeedMsgId] = msg_.getData().current_position;
}

void ArmJoint::CB_ROS_armSpeedCmd(const rover_msgs::msg::ArmMsg& rosMsg_)
{
    _nextArmCmdMsg.data().targetSpeed = rosMsg_.target_speed[_rosArmSpeedMsgId];
}

void ArmJoint::CB_ROS_canSend(void)
{
    this->sendMsg(_nextArmCmdMsg);
}
