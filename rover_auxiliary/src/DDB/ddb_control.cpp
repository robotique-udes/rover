#include "ddb_control_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DDBControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

/**
 * @brief Construct a new DDBControlNode::DDBControlNode object
 *
 */
DDBControlNode::DDBControlNode():
    Node("ddb_control")
{
    _srv_control_bank0 = this->create_service<rover_msgs::srv::DDBControl>(
        "/rover/auxiliary/ddb_control_bank0",
        [this](const std::shared_ptr<rover_msgs::srv::DDBControl::Request> request_,
               std::shared_ptr<rover_msgs::srv::DDBControl::Response> response_)
        {
            if (!request_ || !response_)
            {
                RCLCPP_FATAL(this->get_logger(), "NULL request or response received.");
                return;
            }
            this->ddbControlBank0(*request_, *response_);
        });

    _srv_control_bank1 = this->create_service<rover_msgs::srv::DDBControl>(
        "/rover/auxiliary/ddb_control_bank1",
        [this](const std::shared_ptr<rover_msgs::srv::DDBControl::Request> request_,
               std::shared_ptr<rover_msgs::srv::DDBControl::Response> response_)
        {
            if (!request_ || !response_)
            {
                RCLCPP_FATAL(this->get_logger(), "NULL request or response received.");
                return;
            }
            this->ddbControlBank1(*request_, *response_);
        });
}

/**
 * @brief Decides what to do depending on the user requested command for bank 0
 *
 * @param request_
 * @param response_
 */
void DDBControlNode::ddbControlBank0(const rover_msgs::srv::DDBControl::Request& request_,
                                     rover_msgs::srv::DDBControl::Response& response_)
{
    if (request_.channel_id >= MAX_CHANNELS)
    {
        RCLCPP_WARN(this->get_logger(), "Invalid channel (0 to 3). Received channel: %d", request_.channel_id);
        response_.success = false;
        response_.status = "Invalid channel (0 to 3). Received channel: " + std::to_string(request_.channel_id);
        return;
    }

    switch (request_.command)
    {
        case rover_msgs::srv::DDBControl::Request::TOGGLE_CHANNEL:
            this->stateLogic(request_, response_);
            break;

        case rover_msgs::srv::DDBControl::Request::TOGGLE_MODE:
            this->modeLogic(request_, response_);

            if (_channelInfo[request_.channel_id].mode == eToggleMode::PWM
                && this->valuesCheck(request_.duty_cycle, request_.frequency, response_))
            {
                this->modifyPWM(request_.duty_cycle, request_.frequency, request_.channel_id);
            }
            break;

        case rover_msgs::srv::DDBControl::Request::CHANGE_VALUES:
            if (valuesCheck(request_.duty_cycle, request_.frequency, response_))
            {
                this->valuesLogic(request_, response_);
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Invalid frequency or duty cycle. Check logs for details.");
                response_.success = false;
                response_.status = "Invalid frequency or duty cycle. Check logs for details.";
            }
            break;

        default:
            RCLCPP_WARN(this->get_logger(), "Received request without a valid command: %d", request_.command);
            response_.success = false;
            response_.status = "Received request without a valid command" + std::to_string(request_.command);
    }
}

/**
 * @brief Decides what to do depending on the user requested command for bank 1
 *
 * @param request_
 * @param response_
 */
void DDBControlNode::ddbControlBank1(const rover_msgs::srv::DDBControl::Request& request_,
                                     rover_msgs::srv::DDBControl::Response& response_)
{
    if (request_.channel_id >= MAX_CHANNELS)
    {
        RCLCPP_WARN(this->get_logger(), "Invalid channel (0 to 3). Received channel: %d", request_.channel_id);
        response_.success = false;
        response_.status = "Invalid channel (0 to 3). Received channel: " + std::to_string(request_.channel_id);
        return;
    }

    if (request_.command == rover_msgs::srv::DDBControl::Request::TOGGLE_CHANNEL)
    {
        this->stateLogic(request_, response_);
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Received request without a valid command: %d", request_.command);
        response_.success = false;
        response_.status = "Received request without a valid command: " + std::to_string(request_.command);
    }
}

/**
 * @brief Changes the state to ON or OFF based on the current state.
 *
 * @param channelID_ Received from the user service call
 * @return true if successfully changed the state of the desired channel else
 * @return false
 */
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

/**
 * @brief Changes the mode to FIX or PWM based on the current mode.
 *
 * @param channelID_ Received from the user service call
 * @return true
 * @return false
 */
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

/**
 * @brief Changes the values for the PWM mode when requested.
 *
 * @param dutyCycle_ Value between 0 and 100 received from user service call
 * @param frequency_ Value higher than 0 received from user service call
 * @param channelID_ Received from user service call
 * @return true
 * @return false
 */
bool DDBControlNode::modifyPWM(uint8_t dutyCycle_, float frequency_, uint8_t channelID_)
{
    bool isUpdated = false;

    float oldFrequency = _channelInfo[channelID_].frequency;
    uint8_t oldDutyCycle = _channelInfo[channelID_].dutyCycle;

    _channelInfo[channelID_].frequency = frequency_;
    _channelInfo[channelID_].dutyCycle = dutyCycle_;

    if (_channelInfo[channelID_].frequency == oldFrequency)
    {
        RCLCPP_WARN(this->get_logger(), "Received the same frequency of could not change it for channel %d", channelID_);
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Frequency set to %f", _channelInfo[channelID_].frequency);
        isUpdated = true;
    }

    if (_channelInfo[channelID_].dutyCycle == oldDutyCycle)
    {
        RCLCPP_WARN(this->get_logger(), "Received the same duty cycle of could not change it for channel %d", channelID_);
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Duty cycle set to %d", _channelInfo[channelID_].dutyCycle);
        isUpdated = true;
    }

    return isUpdated;
}

/**
 * @brief Takes the current mode and changes it for a std::string
 *
 * @param mode_ Current mode of the channel
 * @return std::string corresponding to the mode asked
 */
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

/**
 * @brief Main logic for switching state of the desired channel
 *
 * @param request_
 * @param response_
 */
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

/**
 * @brief Main logic for switching mode of the desired channel. Only in bank 0
 *
 * @param request_
 * @param response_
 */
void DDBControlNode::modeLogic(const rover_msgs::srv::DDBControl::Request& request_,
                               rover_msgs::srv::DDBControl::Response& response_)
{
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

/**
 * @brief Checks if the user entered valid values for the duty cycle (between 0 and 100) and frequency (cannot be 0)
 *
 * @param dutyCycle_ Percentage value received from the user service call
 * @param frequency_ Hertz value received from the user service call
 * @param response_
 * @return true if both checks are valid else
 * @return false if either check isn't valid
 */
bool DDBControlNode::valuesCheck(uint8_t dutyCycle_, float frequency_, rover_msgs::srv::DDBControl::Response& response_)
{
    if (dutyCycle_ > 100)
    {
        RCLCPP_ERROR(this->get_logger(), "Duty cycle must be specified as percentage (0 to 100). DC received: %d", dutyCycle_);
        response_.success = false;
        response_.status = "Duty cycle must be specified as percentage (0 to 100). DC received: " + std::to_string(dutyCycle_);
        return false;
    }

    if (frequency_ == 0.0)
    {
        RCLCPP_ERROR(this->get_logger(), "Frequency must be higher than 0 Hz. Frequency received: %f", frequency_);
        response_.success = false;
        response_.status = "Frequency must be higher than 0 Hz. Frequency received: " + std::to_string(frequency_);
        return false;
    }

    return true;
}

/**
 * @brief Main logic for changing PWM values based on the current mode of desired channel
 *
 * @param request_
 * @param response_
 */
void DDBControlNode::valuesLogic(const rover_msgs::srv::DDBControl::Request& request_,
                                 rover_msgs::srv::DDBControl::Response& response_)
{
    switch (_channelInfo[request_.channel_id].mode)
    {
        case eToggleMode::FIX:
            RCLCPP_ERROR(this->get_logger(), "Current mode doesn't allow changing PWM values");
            break;

        case eToggleMode::PWM:
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
            break;
    }
}