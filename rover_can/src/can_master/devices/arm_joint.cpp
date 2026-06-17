#include "arm_joint.hpp"
#include <rover_msgs/msg/detail/arm_msg__struct.hpp>
#include <rover_lib2/helpers/constants.hpp>

ArmJoint::ArmJoint(RoverCan2::Constant::eDeviceId deviceId_,
                   uint8_t rosArmSpeedMsgId_,
                   std::shared_ptr<CanMaster::SharedRosMsg<rover_msgs::msg::ArmMsg>> rosSharedMsg_):
    DeviceT(deviceId_,
            RoverCan2::Publisher<RoverCan2::Msgs::ArmJointCmd>(),
            RoverCan2::SubscriberMember(*this, &ArmJoint::CB_CAN_armPostitionStatus),
            RoverCan2::Publisher<RoverCan2::Msgs::ArmJointConfig>()),
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
    _srv_ArmJointsConfig = this->getAttachedNode()->create_service<rover_msgs::srv::ArmJointConfig>(
        ARM_JOINTS_CONFIG_SERVICE_NAME,
        [this](const std::shared_ptr<rover_msgs::srv::ArmJointConfig::Request> request_,
               std::shared_ptr<rover_msgs::srv::ArmJointConfig::Response> response_)
        {
            this->CB_SRV_armJointsConfig(request_, response_);
        });

    _sub_MorseCodeInput
        = this->getAttachedNode()->create_subscription<rover_msgs::msg::MorseCode>(MORSE_CODE_TOPIC,
                                                                                QOS_DEFAULT,
                                                                                [this](const rover_msgs::msg::MorseCode& rosMsg_)
                                                                                {
                                                                                    this->CB_ROS_mordeCodeInput(rosMsg_);
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

    if (_srv_ArmJointsConfig)
    {
        _srv_ArmJointsConfig.reset();
    }
}

std::vector<RoverCan2::Constant::eDeviceId> ArmJoint::getManagedDevicesIds(void)
{
    return {this->getCanId()};
}

void ArmJoint::CB_CAN_armPostitionStatus(const RoverCan2::Msgs::ArmJointStatus& msg_)
{
    _rosSharedMsg->get().getThreadSafeAccess().current_position[_rosArmSpeedMsgId] = msg_.getData().currentPosition;
    _rosSharedMsg->get().getThreadSafeAccess().current_speed[_rosArmSpeedMsgId] = msg_.getData().currentSpeed;
}

void ArmJoint::CB_ROS_armSpeedCmd(const rover_msgs::msg::ArmMsg& rosMsg_)
{
    _nextArmCmdMsg.data().targetSpeed = rosMsg_.target_speed[_rosArmSpeedMsgId];
}

void ArmJoint::CB_SRV_armJointsConfig(const std::shared_ptr<rover_msgs::srv::ArmJointConfig::Request> request_,
                                      std::shared_ptr<rover_msgs::srv::ArmJointConfig::Response> response_)
{
    if (std::find(VALID_IDS.begin(), VALID_IDS.end(), request_->can_id) == VALID_IDS.end())
    {
        response_->success = false;
        response_->message = "Invalid can id";
    }
    else if (request_->can_id == std::to_underlying(this->getCanId()))
    {
        _nextArmConfigMsg.data().upperLimit = request_->upper_limit;
        _nextArmConfigMsg.data().lowerLimit = request_->lower_limit;
        _nextArmConfigMsg.data().maxSpeed = request_->max_speed;
        _nextArmConfigMsg.data().kpSpeed = request_->kp_speed;
        _nextArmConfigMsg.data().kiSpeed = request_->ki_speed;
        _nextArmConfigMsg.data().kdSpeed = request_->kd_speed;

        eReturnValue result = this->sendMsg(_nextArmConfigMsg);

        switch (result)
        {
            case eReturnValue::SUCCESS:
                response_->success = true;
                response_->message = "Message sent";
                break;
            case eReturnValue::FAILED:
                response_->success = false;
                response_->message = "Failed to send the message";
                break;
            case eReturnValue::NOT_CONCERNED:
                response_->success = false;
                response_->message = "No message sent, not concerned";
                break;
            default:
                response_->success = false;
                response_->message = "Unknown error";
                break;
        }
    }
}

void ArmJoint::CB_ROS_mordeCodeInput(const rover_msgs::msg::MorseCode& msg_)
{
    this->_nextMorseInputMsg.data().symbol = msg_.symbol;
    this->_nextMorseInputMsg.data().speed_wpm = msg_.speed_wpm;
}

void ArmJoint::CB_ROS_canSend(void)
{
    this->sendMsg(_nextArmCmdMsg);
}
