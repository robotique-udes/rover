#ifndef PROPULSION_MOTORS_HPP
#define PROPULSION_MOTORS_HPP

#include "can_master/master_device.hpp"
#include "can_master/shared_msg.hpp"

#include <rover_can2/msgs/prop_speed_cmd.hpp>
#include <rover_can2/msgs/prop_speed_status.hpp>
#include <rover_msgs/msg/propulsion_motor.hpp>

#include <rover_can2/rover_can2.hpp>

class PropulsionMotor : public RoverCan2::Device<RoverCan2::Publisher<RoverCan2::Msgs::PropSpeedCmd>,
                                                 RoverCan2::SubscriberMember<RoverCan2::Msgs::PropSpeedStatus, PropulsionMotor>>,
                        public MasterDevice
{
    using DeviceT = RoverCan2::Device<RoverCan2::Publisher<RoverCan2::Msgs::PropSpeedCmd>,
                                      RoverCan2::SubscriberMember<RoverCan2::Msgs::PropSpeedStatus, PropulsionMotor>>;

    static constexpr const char* PROPULSION_MOTOR_CMD_TOPIC = "/rover/drive_train/wheels_cmd";
    static constexpr const char* PROPULSION_MOTOR_STATUS_TOPIC = "/rover/drive_train/wheels_status";
    static constexpr float PROPULSION_MOTOR_STATUS_PUBLISH_FREQUENCY_HZ = 100.0F;

    static constexpr float CAN_PUBLISH_FREQUENCY = 20.0F;
    static constexpr uint32_t CAN_PUBLISH_PERIOD_MS = static_cast<uint32_t>(ROUND(1'000.0F / CAN_PUBLISH_FREQUENCY));

  public:
    PropulsionMotor(RoverCan2::Constant::eDeviceId deviceId_,
                    uint8_t rosPropSpeedMsgId_,
                    std::shared_ptr<CanMaster::SharedRosMsg<rover_msgs::msg::PropulsionMotor>> rosSharedMsg_);

  private:
    void rosElementInit(void) override;
    void rosElementClean(void) override;
    std::vector<RoverCan2::Constant::eDeviceId> getManagedDevicesIds(void) override;

    void CB_CAN_propSpeedStatus(const RoverCan2::Msgs::PropSpeedStatus& msg_);
    void CB_ROS_propSpeedCmd(const rover_msgs::msg::PropulsionMotor& rosMsg_);
    void CB_ROS_canSend(void);

    const uint8_t _rosPropSpeedMsgId;

    std::shared_ptr<CanMaster::SharedRosMsg<rover_msgs::msg::PropulsionMotor>> _rosSharedMsg;
    RoverCan2::Msgs::PropSpeedCmd _nextPropCmdMsg;

    rclcpp::Publisher<rover_msgs::msg::PropulsionMotor>::SharedPtr _pub_MotorStatus;
    rclcpp::Subscription<rover_msgs::msg::PropulsionMotor>::SharedPtr _sub_MotorStatus;
    rclcpp::TimerBase::SharedPtr _timerCanSend;
};

#endif  // PROPULSIONMOTORS_HPP
