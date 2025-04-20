#ifndef __DDB_NODE_HPP__
#define __DDB_NODE_HPP__

#include "rovus_lib/camera_info.hpp"
#include "rovus_lib/macros.h"

#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/srv/ddb_control.hpp"

class DDBControlNode : public rclcpp::Node
{
    static constexpr size_t MAX_CHANNELS = 4UL;

    enum class eOutputState : uint8_t
    {
        OFF = rover_msgs::srv::DDBControl::Request::OUTPUT_OFF,
        ON = rover_msgs::srv::DDBControl::Request::OUTPUT_ON,
        PWM = rover_msgs::srv::DDBControl::Request::OUTPUT_PWM,
    };

    struct sChannelInfo
    {
        eOutputState state = eOutputState::ON;
        float dutyCycle = 0.0;
        float frequency = 0.0;
    };

  public:
    DDBControlNode();
    ~DDBControlNode() = default;

  private:
    void callbackDdbControlBank0(const rover_msgs::srv::DDBControl::Request& request_,
                                 rover_msgs::srv::DDBControl::Response& response_);
    void callbackDdbControlBank1(const rover_msgs::srv::DDBControl::Request& request_,
                                 rover_msgs::srv::DDBControl::Response& response_);
    void setStateLogic(const rover_msgs::srv::DDBControl::Request& request_, rover_msgs::srv::DDBControl::Response& response_);
    void setValuesLogic(const rover_msgs::srv::DDBControl::Request& request_, rover_msgs::srv::DDBControl::Response& response_);

    bool setChannelOutput(uint8_t channelID_, eOutputState desiredState_);
    bool setPWMValues(float dutyCycle_, float frequency_, uint8_t channelID_);
    bool valuesCheck(float dutyCycle_, float frequency_, uint8_t channelID_, rover_msgs::srv::DDBControl::Response& response_);

    rclcpp::Service<rover_msgs::srv::DDBControl>::SharedPtr _srv_control_bank0;
    rclcpp::Service<rover_msgs::srv::DDBControl>::SharedPtr _srv_control_bank1;

    sChannelInfo _channelInfo[MAX_CHANNELS] = {};
};

#endif  // __DDB_NODE_HPP__
