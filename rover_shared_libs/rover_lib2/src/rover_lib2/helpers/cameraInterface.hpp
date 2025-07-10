#ifndef CAMERA_INTERFACE_HPP
#define CAMERA_INTERFACE_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <rclcpp/subscription.hpp>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/camera_control.hpp"

class CameraInterface
{
    static constexpr float SEND_COMMAND_PTZ_FREQUENCY = 5.F;
    static constexpr float SEND_COMMAND_POWER_FREQUENCY = 0.5F;
    static constexpr float SEND_CONFIG_PTZ_FREQUENCY = 0.5F;
    static constexpr float RECEIVE_STATUS_FREQUENCY = 0.5F;

    static constexpr const char* POWER_STATUS_TOPIC = "/rover/camera/power_status";
    static constexpr const char* PTZ_STATUS_TOPIC = "/rover/camera/PTZ_status";

    static constexpr uint8_t NUMBER_CAM = 5;
    static constexpr float GOAL_MARGIN = 0.1;

#warning unused

#warning IF STATUS IS NOT RECEIVED A MESSAGE IS CREATED WITH ALL 0 (default) WE WOULD WANT TO MESSAGE IN THAT CASE...

#warning WE WOULD WHAT TO BE ABLE TO HAVE A FCT TO STOP THE PUBLISHMENT...

  public:
    CameraInterface(std::shared_ptr<rclcpp::Node> node_,
                    const std::string& ptzCommandTopic_,
                    const std::string& ptzConfigTopic_,
                    const std::string& powerCommandTopic_);

    void setPTZCmd(rover_msgs::msg::CameraControl goalMsg_, size_t id_);
    rover_msgs::msg::CameraControl getPTZCmd(size_t id_) const;

    void setPTZConfig(rover_msgs::msg::CameraControl goalMsg_, size_t id_);
    rover_msgs::msg::CameraControl getPtzConfig(size_t id_) const;

    void setPowerCmd(rover_msgs::msg::CameraControl goalMsg_, size_t id_);
    rover_msgs::msg::CameraControl getPowerCmd(size_t id_) const;

    void forgetPTZCmd(size_t id_);
    void forgetPTZConfig(size_t id_);
    void forgetPowerCmd(size_t id_);

    rover_msgs::msg::CameraControl getLastPowerStatusMsg(size_t id_) const;
    rover_msgs::msg::CameraControl getLastPtzStatusMsg(size_t id_) const;

    bool isGoalReached(size_t id_);

  private:
    void CB_publishPtzCmd(void);
    void CB_publishPtzConfig(void);
    void CB_publishPowerCmd(void);

    void CB_subscriberPowerStatus(rover_msgs::msg::CameraControl statusMsg_);
    void CB_subscriberPtzStatus(rover_msgs::msg::CameraControl statusMsg_);

    rclcpp::Publisher<rover_msgs::msg::CameraControl>::SharedPtr _pub_PTZCmd;
    rclcpp::Publisher<rover_msgs::msg::CameraControl>::SharedPtr _pub_configCmd;
    rclcpp::Publisher<rover_msgs::msg::CameraControl>::SharedPtr _pub_powerCmd;
    rclcpp::Subscription<rover_msgs::msg::CameraControl>::SharedPtr _sub_powerStatus;
    rclcpp::Subscription<rover_msgs::msg::CameraControl>::SharedPtr _sub_PTZStatus;


    rclcpp::TimerBase::SharedPtr _timer_pubPTZCmd;
    rclcpp::TimerBase::SharedPtr _timer_pubPTZConfig;
    rclcpp::TimerBase::SharedPtr _timer_pubPowerCmd;

    std::array<rover_msgs::msg::CameraControl, NUMBER_CAM> _lastPtzCmdMsg;
    std::array<rover_msgs::msg::CameraControl, NUMBER_CAM> _lastPtzConfigMsg;
    std::array<rover_msgs::msg::CameraControl, NUMBER_CAM> _lastPowerMsg;

    std::array<bool, NUMBER_CAM> _isCamConcerned;

    std::array<rover_msgs::msg::CameraControl, NUMBER_CAM> _lastPowerStatusMsg;
    std::array<rover_msgs::msg::CameraControl, NUMBER_CAM> _lastPtzStatusMsg;

    std::array<bool, NUMBER_CAM> _isGoalReached = {false};

    std::shared_ptr<rclcpp::Node> _node;
};

#endif  // CAMERA_INTERFACE_HPP