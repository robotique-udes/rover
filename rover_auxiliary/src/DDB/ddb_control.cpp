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
    if (request_.on_off == rover_msgs::srv::DDBControl::Request::TOGGLE_SWITCH)
    {
        this->toggleSwitch(request_); 
    }

    if(request_.toggle_mode == rover_msgs::srv::DDBControl::Request::TOGGLE_PWM)
    {
        this->togglePWM(request_);
    }

    if(request_.frequency > 0 && request_.duty_cycle > 0)
    {
        this->modifyPWM(request_.duty_cycle, request_.frequency);
    }


}

void DDBControlNode::toggleSwitch(const rover_msgs::srv::DDBControl::Request& request_)
{

    eSwitchState wantedState;

    if (_currentSwitchState == eSwitchState::OFF)
    {
        wantedState = eSwitchState::ON;
    }
    else if (_currentSwitchState == eSwitchState::ON)
    {
        wantedState = eSwitchState::OFF;
    }

    switch (wantedState)
    {
        case eSwitchState::ON:
            _currentSwitchState = eSwitchState::ON;
            RCLCPP_INFO(this->get_logger(), "Switch #%d turned ON", request_.switch_id);
            break;

        case eSwitchState::OFF:
            _currentSwitchState = eSwitchState::OFF;
            RCLCPP_INFO(this->get_logger(), "Switch #%d turned OFF", request_.switch_id);
            break;
    }
}

void DDBControlNode::togglePWM(const rover_msgs::srv::DDBControl::Request& request_)
{
    eSwitchState wantedMode;
    std::string currentModeStr;

    if (_currentPWMMode == eSwitchState::OFF)
    {
        wantedMode= eSwitchState::ON;
    }
    else if (_currentPWMMode == eSwitchState::ON)
    {
        wantedMode= eSwitchState::OFF;
    }

    switch (wantedMode)
    {
        case eSwitchState::ON:
            _currentPWMMode = eSwitchState::ON;
            if (_currentPWMMode == eSwitchState::ON)
            {
                currentModeStr = "PWM";
            }
            RCLCPP_INFO(this->get_logger(), "Current mode: %s", currentModeStr.c_str());
            break;

        case eSwitchState::OFF:
            _currentPWMMode = eSwitchState::OFF;
            if (_currentPWMMode == eSwitchState::ON)
            {
                currentModeStr = "Normal";
            }
            RCLCPP_INFO(this->get_logger(), "Current mode: %s", currentModeStr.c_str());
            break;
    }
}

void DDBControlNode::modifyPWM(uint8_t duty_cycle, uint8_t frequency)
{
    
}