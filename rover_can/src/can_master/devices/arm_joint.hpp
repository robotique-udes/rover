#ifndef ARM_JOINT_HPP
#define ARM_JOINT_HPP

#include "can_master/master_device.hpp"
#include "can_master/shared_msg.hpp"

#include <rover_can2/msgs/arm_speed_cmd.hpp>
#include <rover_can2/rover_can2.hpp>
#include <rover_can2/msgs/arm_position_status.hpp>
#include <rover_msgs/msg/arm_msg.hpp>

#include <memory>
#include <vector>

class ArmJoint : public RoverCan2::Device<RoverCan2::Publisher<RoverCan2::Msgs::ArmSpeedCmd>,
                                          RoverCan2::SubscriberMember<RoverCan2::Msgs::ArmPositionStatus, ArmJoint>>,
                 public MasterDevice
{
    using DeviceT = RoverCan2::Device<RoverCan2::Publisher<RoverCan2::Msgs::ArmSpeedCmd>,
                                      RoverCan2::SubscriberMember<RoverCan2::Msgs::ArmPositionStatus, ArmJoint>>;

    static constexpr const char* ARM_CMD_TOPIC = "/rover/arm/joints_cmd";
    static constexpr const char* ARM_POSITION_STATUS_TOPIC = "/rover/arm/joints_status";
    static constexpr float ARM_POSITION_STATUS_PUBLISH_FREQUENCY_HZ = 20.0F;
    static constexpr uint32_t CAN_PUBLISH_PERIOD_MS
        = static_cast<uint32_t>(ROUND(1'000.0F / ARM_POSITION_STATUS_PUBLISH_FREQUENCY_HZ));

  public:
    ArmJoint(RoverCan2::Constant::eDeviceId deviceId_,
             uint8_t rosArmSpeedMsgId_,
             std::shared_ptr<CanMaster::SharedRosMsg<rover_msgs::msg::ArmMsg>> rosSharedMsg_);

  private:
    void rosElementInit(void) override;
    void rosElementClean(void) override;
    std::vector<RoverCan2::Constant::eDeviceId> getManagedDevicesIds(void) override;

    void CB_CAN_armPostitionStatus(const RoverCan2::Msgs::ArmPositionStatus& msg_);
    void CB_ROS_armSpeedCmd(const rover_msgs::msg::ArmMsg& rosMsg_);
    void CB_ROS_canSend(void);

    const uint8_t _rosArmSpeedMsgId;
    std::shared_ptr<CanMaster::SharedRosMsg<rover_msgs::msg::ArmMsg>> _rosSharedMsg;
    RoverCan2::Msgs::ArmSpeedCmd _nextArmCmdMsg;

    rclcpp::Publisher<rover_msgs::msg::ArmMsg>::SharedPtr _pub_ArmPositionStatus;
    rclcpp::Subscription<rover_msgs::msg::ArmMsg>::SharedPtr _sub_ArmPositionStatus;
    rclcpp::TimerBase::SharedPtr _timerCanSend;
};

#endif  // ARM_JOINT_HPP