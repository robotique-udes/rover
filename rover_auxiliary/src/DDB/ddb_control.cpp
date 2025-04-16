#include "ddb_control_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DDBControlNode>();
    // rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

DDBControlNode::DDBControlNode():
    Node("ddb_control")
{
    _srv_control = this->create_service<rover_msgs::srv::DDBControl>(
        "/rover/auxiliary/ddb_control",
        [this](const std::shared_ptr<rover_msgs::srv::DDBControl::Request> request_,
               std::shared_ptr<rover_msgs::srv::DDBControl::Response> response_)
        {
            if (!request_ || !response_)
            {
                RCLCPP_FATAL(this->get_logger(), "NULL request or response received.");
                return;
            }
            this->ddbControl(*request_, *response_);
        });
}

void DDBControlNode::ddbControl(const rover_msgs::srv::DDBControl::Request& request_,
                                rover_msgs::srv::DDBControl::Response& response_)
{

    if(request_.switch_id >= 8 || request_.switch_id < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid switch ID: %d", request_.switch_id);
        response_.success = false;
        response_.status = "Invalid switch ID";
        return;
    }

    if (request_.on_off == rover_msgs::srv::DDBControl::Request::TOGGLE_SWITCH)
    {
        this->toggleSwitch(request_.switch_id);
    }

    if (request_.toggle_mode == rover_msgs::srv::DDBControl::Request::TOGGLE_PWM && request_.switch_id < 4)
    {
        this->togglePWM(request_.switch_id);

        if (request_.frequency >= 0 && request_.duty_cycle >= 0)
        {
            this->modifyPWM(request_.duty_cycle, request_.frequency, request_.switch_id);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid frequency or duty cycle");
            response_.success = false;
            response_.status = "Invalid frequency or duty cycle";
            return;
        }
    }
}

void DDBControlNode::toggleSwitch(uint8_t switchID_)
{
    eToggleState wantedState;

    if (_switchInfo[switchID_].state == eToggleState::OFF)
    {
        wantedState = eToggleState::ON;
    }
    else if (_switchInfo[switchID_].state == eToggleState::ON)
    {
        wantedState = eToggleState::OFF;
    }

    switch (wantedState)
    {
        case eToggleState::ON:
            _switchInfo[switchID_].state = eToggleState::ON;
            RCLCPP_INFO(this->get_logger(), "Switch #%d turned ON", switchID_);
            break;

        case eToggleState::OFF:
            _switchInfo[switchID_].state = eToggleState::OFF;
            RCLCPP_INFO(this->get_logger(), "Switch #%d turned OFF", switchID_);
            break;
    }
}

void DDBControlNode::togglePWM(uint8_t switchID_)
{
    eToggleMode wantedMode = eToggleMode::FIX;

    if (_switchInfo[switchID_].mode == eToggleMode::FIX)
    {
        wantedMode = eToggleMode::PWM;
    }
    else if (_switchInfo[switchID_].mode == eToggleMode::PWM)
    {
        wantedMode = eToggleMode::FIX;
    }

    switch (wantedMode)
    {
        case eToggleMode::PWM:
            _switchInfo[switchID_].mode = eToggleMode::PWM;
            RCLCPP_INFO(this->get_logger(), "Current mode: %d", _switchInfo[switchID_].mode);  // Not good
            break;

        case eToggleMode::FIX:
            _switchInfo[switchID_].mode = eToggleMode::FIX;
            RCLCPP_INFO(this->get_logger(), "Current mode: %d", _switchInfo[switchID_].mode);  // Not good
            break;
    }
}

void DDBControlNode::modifyPWM(uint8_t dutyCycle_, uint8_t frequency_, uint8_t switchID_) 
{
    _switchInfo[switchID_].frequency = frequency_;
    _switchInfo[switchID_].dutyCycle = dutyCycle_;
}