#ifndef __DDB__NODE__HPP__
#define __DDB__NODE__HPP__

#include "rovus_lib/camera_info.hpp"
#include "rovus_lib/macros.h"

#include "rclcpp/rclcpp.hpp"
#include <rover_msgs/srv/ddb_control.hpp>

class DDBControlNode : public rclcpp::Node
{
    static constexpr size_t MAX_CHANNELS = 8;

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

    struct sChannelInfo
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
    void modeLogic(const rover_msgs::srv::DDBControl::Request& request_, rover_msgs::srv::DDBControl::Response& response_);
    void stateLogic(const rover_msgs::srv::DDBControl::Request& request_, rover_msgs::srv::DDBControl::Response& response_);
    void valuesLogic(const rover_msgs::srv::DDBControl::Request& request_, rover_msgs::srv::DDBControl::Response& response_);

    bool toggleChannel(uint8_t channelID_);
    bool toggleMode(uint8_t channelID_);
    bool modifyPWM(uint8_t dutyCycle_, uint8_t frequency_, uint8_t channelID_);
    std::string toStr(eToggleMode mode_);

    rclcpp::Service<rover_msgs::srv::DDBControl>::SharedPtr _srv_control;

    sChannelInfo _channelInfo[MAX_CHANNELS] = {};
};

#endif