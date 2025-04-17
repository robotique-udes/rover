#include "ddb_control_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DDBControlNode>();
    rclcpp::spin(node);
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
    if (request_.channel_id >= 8)
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid channel ID: %d", request_.channel_id);
        response_.success = false;
        response_.status = "Invalid channel ID";
    }

    if (request_.duty_cycle > 100)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Duty cycle must be specified as percentage (0 to 100). DC received: %d",
                     request_.duty_cycle);
        response_.success = false;
        response_.status
            = "Duty cycle must be specified as percentage (0 to 100). DC received: " + std::to_string(request_.duty_cycle);
    }

    if (request_.frequency == 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Frequency must be higher than 0 Hz. Frequency received: %d", request_.frequency);
        response_.success = false;
        response_.status = "Frequency must be higher than 0 Hz. Frequency received: " + std::to_string(request_.frequency);
    }

    switch (request_.command)
    {
        case rover_msgs::srv::DDBControl::Request::TOGGLE_CHANNEL:
            this->stateLogic(request_, response_);
            break;

        case rover_msgs::srv::DDBControl::Request::TOGGLE_MODE:
            this->modeLogic(request_, response_);

            if (_channelInfo[request_.channel_id].mode == eToggleMode::PWM && (request_.frequency > 0 && request_.duty_cycle > 0))
            {
                this->modifyPWM(request_.duty_cycle, request_.frequency, request_.channel_id);
            }
            break;

        case rover_msgs::srv::DDBControl::Request::CHANGE_VALUES:
            this->valuesLogic(request_, response_);
            break;

        default:
            RCLCPP_WARN(this->get_logger(), "Received request without a valid command");
            response_.success = false;
            response_.status = "Received request without a valid command";
    }
}

bool DDBControlNode::toggleChannel(uint8_t channelID_)
{
    eToggleState wantedState;

    if (_channelInfo[channelID_].state == eToggleState::OFF)
    {
        wantedState = eToggleState::ON;
    }
    else if (_channelInfo[channelID_].state == eToggleState::ON)
    {
        wantedState = eToggleState::OFF;
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid state for channel: %d", channelID_);
        return false;
    }

    switch (wantedState)
    {
        case eToggleState::ON:
            _channelInfo[channelID_].state = eToggleState::ON;
            RCLCPP_INFO(this->get_logger(), "Channel #%d turned ON", channelID_);
            break;

        case eToggleState::OFF:
            _channelInfo[channelID_].state = eToggleState::OFF;
            RCLCPP_INFO(this->get_logger(), "Channel #%d turned OFF", channelID_);
            break;
    }
    return true;
}

bool DDBControlNode::toggleMode(uint8_t channelID_)
{
    eToggleMode wantedMode = eToggleMode::FIX;
    std::string currentMode;

    if (_channelInfo[channelID_].mode == eToggleMode::FIX)
    {
        wantedMode = eToggleMode::PWM;
    }
    else if (_channelInfo[channelID_].mode == eToggleMode::PWM)
    {
        wantedMode = eToggleMode::FIX;
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid mode for channel: %d", channelID_);
        return false;
    }

    switch (wantedMode)
    {
        case eToggleMode::PWM:
            _channelInfo[channelID_].mode = eToggleMode::PWM;
            currentMode = this->toStr(_channelInfo[channelID_].mode);
            RCLCPP_INFO(this->get_logger(), "Current mode: %s", currentMode.c_str());
            break;

        case eToggleMode::FIX:
            _channelInfo[channelID_].mode = eToggleMode::FIX;
            currentMode = this->toStr(_channelInfo[channelID_].mode);
            RCLCPP_INFO(this->get_logger(), "Current mode: %s", currentMode.c_str());
            break;
    }

    return true;
}

bool DDBControlNode::modifyPWM(uint8_t dutyCycle_, uint8_t frequency_, uint8_t channelID_)
{
    bool isUpdated = false;

    uint8_t oldFrequency = _channelInfo[channelID_].frequency;
    uint8_t oldDutyCycle = _channelInfo[channelID_].dutyCycle;

    _channelInfo[channelID_].frequency = frequency_;
    _channelInfo[channelID_].dutyCycle = dutyCycle_;

    if (_channelInfo[channelID_].frequency == oldFrequency)
    {
        RCLCPP_WARN(this->get_logger(), "Received same the same frequency of could not change it for channel %d", channelID_);
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Frequency set to %d", _channelInfo[channelID_].frequency);
        isUpdated = true;
    }

    if (_channelInfo[channelID_].dutyCycle == oldDutyCycle)
    {
        RCLCPP_WARN(this->get_logger(), "Received same the same duty cycle of could not change it for channel %d", channelID_);
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Duty cycle set to %d", _channelInfo[channelID_].dutyCycle);
        isUpdated = true;
    }

    return isUpdated;
}

std::string DDBControlNode::toStr(eToggleMode mode_)
{
    switch (mode_)
    {
        case eToggleMode::FIX:
            return "FIX";
        case eToggleMode::PWM:
            return "PWM";
        default:
            return "UNKNOWN";
    }
}

void DDBControlNode::stateLogic(const rover_msgs::srv::DDBControl::Request& request_,
                                rover_msgs::srv::DDBControl::Response& response_)
{
    if (this->toggleChannel(request_.channel_id))
    {
        RCLCPP_INFO(this->get_logger(), "Channel #%d toggled successfully", request_.channel_id);
        response_.success = true;
        response_.status = "Channel #" + std::to_string(request_.channel_id) + " toggled successfully";
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Could not toggle channel #%d", request_.channel_id);
        response_.success = false;
        response_.status = "Failed to toggle channel #" + std::to_string(request_.channel_id) + ". Unrecognised state.";
    }
}

void DDBControlNode::modeLogic(const rover_msgs::srv::DDBControl::Request& request_,
                               rover_msgs::srv::DDBControl::Response& response_)
{
    if (request_.channel_id > 4)
    {
        RCLCPP_WARN(this->get_logger(), "Channels 4 to 7 can't be put in PWM mode. Received channel: %d", request_.channel_id);
        response_.success = false;
        response_.status = "Channels 4 to 7 can't be put in PWM mode. Received channel: " + std::to_string(request_.channel_id);
        return;
    }

    if (!toggleMode(request_.channel_id))
    {
        RCLCPP_ERROR(this->get_logger(), "Mode could not be toggled for channel #%d", request_.channel_id);
        response_.success = false;
        response_.status = "Mode could not be toggled for channel #" + std::to_string(request_.channel_id);
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Successfully toggled mode for channel #%d", request_.channel_id);
        response_.success = true;
        response_.status = "Successfully toggled mode for channel #" + std::to_string(request_.channel_id);
    }
}

void DDBControlNode::valuesLogic(const rover_msgs::srv::DDBControl::Request& request_,
                                 rover_msgs::srv::DDBControl::Response& response_)
{
    if (this->modifyPWM(request_.duty_cycle, request_.frequency, request_.channel_id))
    {
        RCLCPP_INFO(this->get_logger(), "Values for PWM successfully changed.");
        response_.success = true;
        response_.status = "Values for PWM successfully changed.";
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Values for PWM could not be changed. Check logs for details.");
        response_.success = false;
        response_.status = "Values for PWM could not be changed. Check logs for details.";
    }
}