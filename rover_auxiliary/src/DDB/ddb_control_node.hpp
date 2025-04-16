#ifndef __DDB__NODE__HPP__
#define __DDB__NODE__HPP__

#include "rovus_lib/camera_info.hpp"
#include "rovus_lib/macros.h"

#include "rclcpp/rclcpp.hpp"
#include <rover_msgs/srv/ddb_control.hpp>

class DDBControlNode : public rclcpp::Node
{
    enum class eToggleState : size_t
    {
        OFF = 0,
        ON = 1,
    };
    // Should this be one and only enum class?
    enum class eToggleMode : size_t
    {
        FIX,
        PWM,
    };

    struct sSwitchInfo
    {
        eToggleState state = eToggleState::ON;
        eToggleMode mode = eToggleMode::FIX;
        uint8_t dutyCycle = 0;
        uint8_t frequency = 0;
    };

  public:
    DDBControlNode();
    ~DDBControlNode() = default;

  private:
    void ddbControl(const rover_msgs::srv::DDBControl::Request& request_, rover_msgs::srv::DDBControl::Response& response_);

    void toggleSwitch(uint8_t switchID_);
    void togglePWM(uint8_t switchID_);
    void modifyPWM(uint8_t dutyCycle_, uint8_t frequency_, uint8_t switchID_);

    rclcpp::Service<rover_msgs::srv::DDBControl>::SharedPtr _srv_control;

    struct sSwitchInfo _switchInfo[8];
};

#endif