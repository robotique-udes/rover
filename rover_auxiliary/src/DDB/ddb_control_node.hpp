#ifndef __DDB_NODE_HPP__
#define __DDB_NODE_HPP__

#include <rclcpp/rclcpp.hpp>
#include <rover_msgs/srv/ddb_control.hpp>
#include <rover_msgs/msg/ddb_control.hpp>

class DDBControlNode : public rclcpp::Node
{
    static constexpr const char* TOPIC_CONTROL_BANK_0 = "/rover/auxiliary/ddb_control_bank0";
    static constexpr const char* TOPIC_CONTROL_BANK_1 = "/rover/auxiliary/ddb_control_bank1";
    static constexpr const char* TOPIC_DDB_STATE = "/rover/auxiliary/ddb_outputs_state";
    static constexpr size_t MAX_CHANNELS = 4UL;
    static constexpr uint64_t DELAY_PUBLISHER_MS = 100UL;

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

  private:
    void callbackDdbControlBank0(const rover_msgs::srv::DDBControl::Request& request_,
                                 rover_msgs::srv::DDBControl::Response& response_);
    void callbackDdbControlBank1(const rover_msgs::srv::DDBControl::Request& request_,
                                 rover_msgs::srv::DDBControl::Response& response_);
    void setStateLogic(const rover_msgs::srv::DDBControl::Request& request_, rover_msgs::srv::DDBControl::Response& response_);
    void setValuesLogic(const rover_msgs::srv::DDBControl::Request& request_, rover_msgs::srv::DDBControl::Response& response_);

    void callbackDdbStatus(void) const;

    bool setChannelOutput(uint8_t channelID_, eOutputState desiredState_);
    bool setPWMValues(float dutyCycle_, float frequency_, uint8_t channelID_);
    bool valuesCheck(float dutyCycle_, float frequency_, uint8_t channelID_, rover_msgs::srv::DDBControl::Response& response_) const;

    bool setChannelOutput2(uint8_t channelID_, eOutputState desiredState_);
    void setStateLogic2(const rover_msgs::srv::DDBControl::Request& request_, rover_msgs::srv::DDBControl::Response& response_);

    rclcpp::Service<rover_msgs::srv::DDBControl>::SharedPtr _srv_control_bank0;
    rclcpp::Service<rover_msgs::srv::DDBControl>::SharedPtr _srv_control_bank1;
    rclcpp::Publisher<rover_msgs::msg::DDBControl>::SharedPtr _pub_DDB_status;

    rclcpp::TimerBase::SharedPtr _timer_pub;

    sChannelInfo _channelInfo[MAX_CHANNELS] = {};
    eOutputState _channelInfo2[MAX_CHANNELS] = {};
};

#endif  // __DDB_NODE_HPP__
