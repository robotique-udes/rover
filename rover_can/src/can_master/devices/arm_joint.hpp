#ifndef ARM_JOINT_HPP
#define ARM_JOINT_HPP

#include "can_master/master_device.hpp"
#include "can_master/shared_msg.hpp"

#include <rover_can2/msgs/arm_joint_cmd.hpp>
#include <rover_can2/rover_can2.hpp>
#include <rover_can2/msgs/arm_joint_status.hpp>
#include <rover_can2/msgs/arm_joint_config.hpp>
#include <rover_can2/msgs/morse_input.hpp>
#include <rover_msgs/msg/arm_msg.hpp>
#include <rover_msgs/srv/arm_joint_config.hpp>
#include <rover_msgs/msg/morse_code.hpp>

#include <memory>
#include <vector>

class ArmJoint : public RoverCan2::Device<RoverCan2::Publisher<RoverCan2::Msgs::ArmJointCmd>,
                                          RoverCan2::SubscriberMember<RoverCan2::Msgs::ArmJointStatus, ArmJoint>,
                                          RoverCan2::Publisher<RoverCan2::Msgs::ArmJointConfig>>,
                 public MasterDevice
{
    using DeviceT = RoverCan2::Device<RoverCan2::Publisher<RoverCan2::Msgs::ArmJointCmd>,
                                      RoverCan2::SubscriberMember<RoverCan2::Msgs::ArmJointStatus, ArmJoint>,
                                      RoverCan2::Publisher<RoverCan2::Msgs::ArmJointConfig>>;

    static constexpr const char* ARM_CMD_TOPIC = "/rover/arm/joints_cmd";
    static constexpr const char* ARM_POSITION_STATUS_TOPIC = "/rover/arm/joints_status";
    static constexpr const char* ARM_JOINTS_CONFIG_SERVICE_NAME = "/rover/arm/joints_config";
    static constexpr const char* MORSE_CODE_TOPIC = "/base/gui/morse_code";
    static constexpr float ARM_POSITION_STATUS_PUBLISH_FREQUENCY_HZ = 20.0F;
    static constexpr uint32_t CAN_PUBLISH_PERIOD_MS
        = static_cast<uint32_t>(ROUND(1'000.0F / ARM_POSITION_STATUS_PUBLISH_FREQUENCY_HZ));

    static constexpr std::array<uint16_t, 6> VALID_IDS
        = {std::to_underlying(RoverCan2::Constant::eDeviceId::JL_CONTROLLER),
           std::to_underlying(RoverCan2::Constant::eDeviceId::J1_CONTROLLER),
           std::to_underlying(RoverCan2::Constant::eDeviceId::J2_CONTROLLER),
           std::to_underlying(RoverCan2::Constant::eDeviceId::GRIPPER_CLOSE_CONTROLLER),
           std::to_underlying(RoverCan2::Constant::eDeviceId::GRIPPER_ROT_CONTROLLER),
           std::to_underlying(RoverCan2::Constant::eDeviceId::GRIPPER_TILT_CONTROLLER)};

  public:
    ArmJoint(RoverCan2::Constant::eDeviceId deviceId_,
             uint8_t rosArmSpeedMsgId_,
             std::shared_ptr<CanMaster::SharedRosMsg<rover_msgs::msg::ArmMsg>> rosSharedMsg_);

  private:
    void rosElementInit(void) override;
    void rosElementClean(void) override;
    std::vector<RoverCan2::Constant::eDeviceId> getManagedDevicesIds(void) override;

    void CB_CAN_armPostitionStatus(const RoverCan2::Msgs::ArmJointStatus& msg_);
    void CB_ROS_armSpeedCmd(const rover_msgs::msg::ArmMsg& rosMsg_);
    void CB_SRV_armJointsConfig(const std::shared_ptr<rover_msgs::srv::ArmJointConfig::Request> request_,
                                std::shared_ptr<rover_msgs::srv::ArmJointConfig::Response> response_);
    void CB_ROS_mordeCodeInput(const rover_msgs::msg::MorseCode& msg_);
    void CB_ROS_canSend(void);

    const uint8_t _rosArmSpeedMsgId;
    std::shared_ptr<CanMaster::SharedRosMsg<rover_msgs::msg::ArmMsg>> _rosSharedMsg;
    RoverCan2::Msgs::ArmJointCmd _nextArmCmdMsg;
    RoverCan2::Msgs::ArmJointConfig _nextArmConfigMsg;
    RoverCan2::Msgs::MorseInput _nextMorseInputMsg;

    rclcpp::Publisher<rover_msgs::msg::ArmMsg>::SharedPtr _pub_ArmPositionStatus;
    rclcpp::Subscription<rover_msgs::msg::ArmMsg>::SharedPtr _sub_ArmPositionStatus;
    rclcpp::Subscription<rover_msgs::msg::MorseCode>::SharedPtr _sub_MorseCodeInput;
    rclcpp::Service<rover_msgs::srv::ArmJointConfig>::SharedPtr _srv_ArmJointsConfig;
    rclcpp::TimerBase::SharedPtr _timerCanSend;
};

#endif  // ARM_JOINT_HPP