#ifndef CAMERA_MAIN_HPP
#define CAMERA_MAIN_HPP

#include "can_master/master_device.hpp"

#include <rover_can2/rover_can2.hpp>

#include <rover_can2/msgs/power_cmd.hpp>
#include <rover_can2/msgs/power_status.hpp>

#include <rover_msgs/msg/camera_control.hpp>

class Camera : public RoverCan2::Device<RoverCan2::Publisher<RoverCan2::Msgs::PowerCmd, 1>,
                                        RoverCan2::SubscriberMember<RoverCan2::Msgs::PowerStatus, Camera>>,
               public MasterDevice
{
    static constexpr const char* CAMERA_POWER_STATUS_TOPIC = "/rover/cameras/power_status";
    static constexpr float CAMERA_POWER_STATUS_PUB_FREQ = 2.0F;
    static constexpr uint32_t CAMERA_POWER_STATUS_PUB_PERIOD_MS
        = static_cast<uint32_t>(ROUND(1'000.0F / CAMERA_POWER_STATUS_PUB_FREQ));
    static constexpr const char* CAMERA_POWER_CONTROL_TOPIC = "/rover/cameras/status_infos";

  public:
    Camera(RoverCan2::Constant::eDeviceId IdCan_, uint8_t IdCameraControlMsgCam_);

  private:
    void rosElementInit() override;
    void rosElementClean() override;
    std::vector<RoverCan2::Constant::eDeviceId> getManagedDevicesIds(void) override;

    void CB_CAN_powerStatus(const RoverCan2::Msgs::PowerStatus& canMsg_);
    void CB_ROS_powerCmd(const rover_msgs::msg::CameraControl& rosMsg_);
    void CB_sendCanPowerCmd(void);

    const uint8_t _idCamCameraControlMsg;
    RoverCan2::Msgs::PowerCmd _nextCanMsg;

    rclcpp::Publisher<rover_msgs::msg::CameraControl>::SharedPtr _pub_powerStatus;
    rclcpp::Subscription<rover_msgs::msg::CameraControl>::SharedPtr _sub_powerCmd;
    rclcpp::TimerBase::SharedPtr _timerCanSendPowerCmd;
};

#endif  // CAMERA_MAIN_HPP
