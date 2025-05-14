#include "propulsion_motors.hpp"
#include <rover_msgs/msg/detail/propulsion_motor__struct.hpp>
#include <rover_lib2/helpers/constants.hpp>

PropulsionMotor::PropulsionMotor(RoverCan2::Constant::eDeviceId deviceId_,
                                 uint8_t rosPropSpeedMsgId_,
                                 std::shared_ptr<CanMaster::SharedRosMsg<rover_msgs::msg::PropulsionMotor>> rosSharedMsg_):
    DeviceT(deviceId_,
            RoverCan2::Publisher<RoverCan2::Msgs::PropSpeedCmd>(),
            RoverCan2::SubscriberMember(*this, &PropulsionMotor::CB_CAN_propSpeedStatus)),
    _rosPropSpeedMsgId(rosPropSpeedMsgId_),
    _rosSharedMsg(rosSharedMsg_)
{
    ASSERT_COND_MSG(rosSharedMsg_, "rosSharedMsg_ can't be nullptr");
    ASSERT_COND_MSG_ARGS(_rosPropSpeedMsgId < rover_msgs::msg::PropulsionMotor::MOTOR_MAX,
                         "_rosPropSpeedMsgId (%u) must be in range [0; %u]",
                         _rosPropSpeedMsgId,
                         (rover_msgs::msg::PropulsionMotor::MOTOR_MAX - 1U));
}

void PropulsionMotor::rosElementInit(void)
{
    _pub_MotorStatus
        = this->getAttachedNode()->create_publisher<rover_msgs::msg::PropulsionMotor>(PROPULSION_MOTOR_STATUS_TOPIC, 1);
    _rosSharedMsg->attachNewPub(this->getAttachedNode(), _pub_MotorStatus, PROPULSION_MOTOR_STATUS_PUBLISH_FREQUENCY_HZ);

    _timerCanSend = this->getAttachedNode()->create_wall_timer(std::chrono::milliseconds(CAN_PUBLISH_PERIOD_MS),
                                                               [this](void)
                                                               {
                                                                   this->CB_ROS_canSend();
                                                               });
    _sub_MotorStatus = this->getAttachedNode()->create_subscription<rover_msgs::msg::PropulsionMotor>(
        PROPULSION_MOTOR_CMD_TOPIC,
        QOS_DEFAULT,
        [this](const rover_msgs::msg::PropulsionMotor& rosMsg_)
        {
            this->CB_ROS_propSpeedCmd(rosMsg_);
        });
}

void PropulsionMotor::rosElementClean(void)
{
    if (_pub_MotorStatus)
    {
        _rosSharedMsg->removePub(_pub_MotorStatus);
        _pub_MotorStatus.reset();
    }

    if (_timerCanSend)
    {
        _timerCanSend.reset();
    }

    if (_sub_MotorStatus)
    {
        _sub_MotorStatus.reset();
    }
}

std::vector<RoverCan2::Constant::eDeviceId> PropulsionMotor::getManagedDevicesIds(void)
{
    return std::vector{this->getCanId()};
}

void PropulsionMotor::CB_CAN_propSpeedStatus(const RoverCan2::Msgs::PropSpeedStatus& msg_)
{
    _rosSharedMsg->get().getThreadSafeAccess().current_speed[_rosPropSpeedMsgId] = msg_.getData().current_speed;
}

void PropulsionMotor::CB_ROS_propSpeedCmd(const rover_msgs::msg::PropulsionMotor& rosMsg_)
{
    _nextPropCmdMsg.data().target_speed = rosMsg_.target_speed[_rosPropSpeedMsgId];
}

void PropulsionMotor::CB_ROS_canSend(void)
{
    this->sendMsg(_nextPropCmdMsg);
}
