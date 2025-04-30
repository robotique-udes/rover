#ifndef CAMERA_MAIN_HPP
#define CAMERA_MAIN_HPP

#include "CanMaster/MasterDevice.hpp"

#include <rover_can2/rover_can2.hpp>

#include <rover_can2/msgs/power_cmd.hpp>
#include <rover_can2/msgs/power_status.hpp>

#include <rover_msgs/msg/device_power.hpp>

class CameraMain : public MasterDevice,
                   public RoverCan2::Device<RoverCan2::Publisher<RoverCan2::Msgs::PowerCmd, 1>,
                                            RoverCan2::SubscriberMember<RoverCan2::Msgs::PowerStatus, CameraMain>>
{
  public:
    CameraMain();

  private:
    void rosElementInit() override;
    void rosElementClean() override;

    void CB_CAN_powerStatus(const RoverCan2::Msgs::PowerStatus& canMsg_);
    void CB_ROS_powerCmd(const rover_msgs::msg::DevicePower& rosMsg_);
    void CB_sendCanPowerCmd(void);

    RoverCan2::Msgs::PowerCmd _nextCanMsg;

    rclcpp::Publisher<rover_msgs::msg::DevicePower>::SharedPtr _pub_powerStatus;
    rclcpp::Subscription<rover_msgs::msg::DevicePower>::SharedPtr _sub_powerStatus;
    rclcpp::TimerBase::SharedPtr _timerCanSendPowerCmd;
};

#endif  // CAMERA_MAIN_HPP
