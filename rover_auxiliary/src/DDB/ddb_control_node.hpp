#ifndef __DDB_NODE_HPP__
#define __DDB_NODE_HPP__

#include "rovus_lib/camera_info.hpp"
#include "rovus_lib/macros.h"

#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/srv/ddb_control.hpp"

class DDBControlNode : public rclcpp::Node
{
    static constexpr size_t MAX_CHANNELS = 4UL;

    enum class eOutputState : size_t
    {
        OFF = rover_msgs::srv::DDBControl::Request::OUTPUT_OFF,
        ON = rover_msgs::srv::DDBControl::Request::OUTPUT_ON,
    };

    enum class eOutputMode : size_t
    {
        FIX,
        PWM,
    };

    struct sChannelInfo
    {
        eOutputState state = eOutputState::ON;
        eOutputMode mode = eOutputMode::FIX;
        float dutyCycle = 0.0;
        float frequency = 0.0;
    };

  public:
    DDBControlNode();
    ~DDBControlNode() = default;

  private:
    void ddbControlBank0(const rover_msgs::srv::DDBControl::Request& request_, rover_msgs::srv::DDBControl::Response& response_);
    void ddbControlBank1(const rover_msgs::srv::DDBControl::Request& request_, rover_msgs::srv::DDBControl::Response& response_);
    void setModeLogic(const rover_msgs::srv::DDBControl::Request& request_, rover_msgs::srv::DDBControl::Response& response_);
    void setStateLogic(const rover_msgs::srv::DDBControl::Request& request_, rover_msgs::srv::DDBControl::Response& response_);
    void setValuesLogic(const rover_msgs::srv::DDBControl::Request& request_, rover_msgs::srv::DDBControl::Response& response_);

    bool setChannelOutput(uint8_t channelID_, uint8_t desiredState_);
    bool setOutputMode(uint8_t channelID_, eOutputMode mode_);
    bool setPWMValues(float dutyCycle_, float frequency_, uint8_t channelID_);
    bool valuesCheck(float dutyCycle_, float frequency_, uint8_t channelID_, rover_msgs::srv::DDBControl::Response& response_);
    std::string eOutputModeToStr(eOutputMode mode_);
    uint8_t eOutputStateToUint8_t(eOutputState state_);

    rclcpp::Service<rover_msgs::srv::DDBControl>::SharedPtr _srv_control_bank0;
    rclcpp::Service<rover_msgs::srv::DDBControl>::SharedPtr _srv_control_bank1;

    sChannelInfo _channelInfo[MAX_CHANNELS] = {};
};

#endif // __DDB_NODE_HPP__
