#ifndef CAMERA_MAIN_HPP
#define CAMERA_MAIN_HPP

#include "can_master/master_device.hpp"

#include <rover_can2/rover_can2.hpp>
#include <rover_can2/msgs/power_cmd.hpp>
#include <rover_can2/msgs/power_status.hpp>
#include <rover_can2/msgs/PTZ_cmd.hpp>
#include <rover_can2/msgs/PTZ_config.hpp>
#include <rover_can2/msgs/PTZ_status.hpp>
#include <rover_msgs/msg/camera_control.hpp>
#include <rover_msgs/msg/camera_config.hpp>

class Camera : public RoverCan2::Device<RoverCan2::Publisher<RoverCan2::Msgs::PowerCmd, 1>,
                                        RoverCan2::Publisher<RoverCan2::Msgs::PtzCmd, 1>,
                                        RoverCan2::Publisher<RoverCan2::Msgs::PtzConfig, 1>,
                                        RoverCan2::SubscriberMember<RoverCan2::Msgs::PowerStatus, Camera>,
                                        RoverCan2::SubscriberMember<RoverCan2::Msgs::PtzStatus, Camera>>,
               public MasterDevice
{
    static constexpr const char* TOPIC_CAMERA_POWER_STATUS = "/rover/camera/power_status";
    static constexpr const char* TOPIC_CAMERA_POWER_COMMAND = "/rover/camera/power_cmd/manager";

    static constexpr float CAMERA_POWER_STATUS_PUB_FREQ = .5F;
    static constexpr float CAMERA_POWER_CMD_PUB_FREQ = .5F;

    static constexpr const char* TOPIC_CAMERA_PTZ_STATUS = "/rover/camera/PTZ_status";
    static constexpr const char* TOPIC_CAMERA_PTZ_COMMAND_MANAGER = "/rover/camera/PTZ_cmd/manager";
    static constexpr const char* TOPIC_CAMERA_PTZ_CONFIG_MANAGER = "/rover/camera/PTZ_config/manager";

    static constexpr float CAMERA_PTZ_STATUS_PUB_FREQ = 5.F;
    static constexpr float CAMERA_PTZ_CMD_PUB_FREQ = 5.F;
    static constexpr float CAMERA_PTZ_CONFIG_PUB_FREQ = .5F;

  public:
    Camera(RoverCan2::Constant::eDeviceId IdCan_, uint8_t IdCameraControlMsgCam_);

  private:
    void rosElementInit() override;
    void rosElementClean() override;
    std::vector<RoverCan2::Constant::eDeviceId> getManagedDevicesIds(void) override;

    void CB_CAN_powerStatus(const RoverCan2::Msgs::PowerStatus& canMsg_);
    void CB_CAN_ptzStatus(const RoverCan2::Msgs::PtzStatus& canMsg_);

    void CB_ROS_powerCmd(const rover_msgs::msg::CameraControl& rosMsg_);
    void CB_ROS_ptzCmd(const rover_msgs::msg::CameraControl& rosMsg_);
    void CB_ROS_ptzConfig(const rover_msgs::msg::CameraConfig& rosMsg_);

    void CB_sendCanPowerCmd(void);
    void CB_sendCanPtzCmd(void);
    void CB_sendCanPtzConfig(void);

    const uint8_t _idCamCameraControlMsg;

    RoverCan2::Msgs::PowerCmd _nextPowerCanMsg;
    RoverCan2::Msgs::PtzCmd _nextPtzCmd;
    RoverCan2::Msgs::PtzConfig _nextPtzConfig;

    rclcpp::Publisher<rover_msgs::msg::CameraControl>::SharedPtr _pub_powerStatus;
    rclcpp::Publisher<rover_msgs::msg::CameraControl>::SharedPtr _pub_ptzStatus;

    rclcpp::Subscription<rover_msgs::msg::CameraControl>::SharedPtr _sub_powerCmd;
    rclcpp::Subscription<rover_msgs::msg::CameraControl>::SharedPtr _sub_ptzCmd;
    rclcpp::Subscription<rover_msgs::msg::CameraConfig>::SharedPtr _sub_ptzConfig;

    rclcpp::TimerBase::SharedPtr _timerCanSendPowerCmd;
    rclcpp::TimerBase::SharedPtr _timerCanSendPtzCmd;
    rclcpp::TimerBase::SharedPtr _timerCanSendPtzConfig;
};

#endif  // CAMERA_MAIN_HPP
