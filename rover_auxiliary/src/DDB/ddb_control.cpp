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
 * @brief Tries to apply the user requested command for bank 0
 *
 * @param request_
 * @param response_
 */
void DDBControlNode::ddbControlBank0(const rover_msgs::srv::DDBControl::Request& request_,
                                     rover_msgs::srv::DDBControl::Response& response_)
{
    if (request_.channel_id >= MAX_CHANNELS)
    {
        std::string msg = "Invalid channel, range is [0; " + std::to_string(MAX_CHANNELS - 1UL)
                          + "]. Received channel : " + std::to_string(request_.channel_id);
        RCLCPP_WARN(this->get_logger(), msg.c_str());
        response_.success = false;
        response_.status = msg;
        return;
    }

    switch (request_.output_state)
    {
        case rover_msgs::srv::DDBControl::Request::OUTPUT_ON:
            [[fallthrough]];

        case rover_msgs::srv::DDBControl::Request::OUTPUT_OFF:
            this->setStateLogic(request_, response_);
            this->setModeLogic(request_, response_);
            this->setPWMValues(0.0, 0.0, request_.channel_id);
            break;

        case rover_msgs::srv::DDBControl::Request::OUTPUT_PWM:
            this->setModeLogic(request_, response_);

            if (valuesCheck(request_.duty_cycle, request_.frequency, request_.channel_id, response_))
            {
                this->setValuesLogic(request_, response_);
            }
            else
            {
                std::string msg = "Invalid frequency or duty cycle. Check logs for details.";
                RCLCPP_ERROR(this->get_logger(), msg.c_str());
                response_.success = false;
                response_.current_output_state = eOutputStateToUint8_t(_channelInfo[request_.channel_id].state);
                response_.status = msg;
                return;
            }
            break;

        default:
            std::string msg = "Received request without a valid command" + std::to_string(request_.output_state);
            RCLCPP_WARN(this->get_logger(), msg.c_str());
            response_.success = false;
            response_.current_output_state = eOutputStateToUint8_t(_channelInfo[request_.channel_id].state);
            response_.status = msg;
    }
}

/**
 * @brief Tries to apply the user requested command for bank 1
 *
 * @param request_
 * @param response_
 */
void DDBControlNode::ddbControlBank1(const rover_msgs::srv::DDBControl::Request& request_,
                                     rover_msgs::srv::DDBControl::Response& response_)
{
    if (request_.channel_id >= MAX_CHANNELS)
    {
        std::string msg = "Invalid channel, range is [0; " + std::to_string(MAX_CHANNELS - 1UL)
                          + "]. Received channel : " + std::to_string(request_.channel_id);
        RCLCPP_WARN(this->get_logger(), msg.c_str());
        response_.success = false;
        response_.status = msg;
        return;
    }

    if (request_.output_state == rover_msgs::srv::DDBControl::Request::OUTPUT_ON
        || request_.output_state == rover_msgs::srv::DDBControl::Request::OUTPUT_OFF)
    {
        this->setStateLogic(request_, response_);
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Received request without a valid command: %d", request_.output_state);
        response_.success = false;
        response_.current_output_state = eOutputStateToUint8_t(_channelInfo[request_.channel_id].state);
        response_.status = "Received request without a valid command: " + std::to_string(request_.output_state);
    }
}

/**
 * @brief Changes the state to ON or OFF based on the current state.
 *
 * @param channelID_ Received from the user service call
 * @return true if successfully changed the state of the desired channel else
 * @return false
 */
bool DDBControlNode::setChannelOutput(uint8_t channelID_, uint8_t desiredState_)
{
    eOutputState wantedState = TO_UNDERLYING(desiredState_);

    if (desiredState_ == rover_msgs::srv::DDBControl::Request::OUTPUT_ON)
    {
        wantedState = eOutputState::ON;
    }
    else if (desiredState_ == rover_msgs::srv::DDBControl::Request::OUTPUT_OFF)
    {
        wantedState = eOutputState::OFF;
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid state for channel: %d", channelID_);
        return false;
    }

    switch (wantedState)
    {
        case eOutputState::ON:
            _channelInfo[channelID_].state = eOutputState::ON;
            RCLCPP_INFO(this->get_logger(), "Set channel #%d output as ON", channelID_);
            break;

        case eOutputState::OFF:
            _channelInfo[channelID_].state = eOutputState::OFF;
            RCLCPP_INFO(this->get_logger(), "Set channel #%d output as OFF", channelID_);
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
bool DDBControlNode::setOutputMode(uint8_t channelID_, eOutputMode desiredMode_)
{
    std::string currentMode;

    switch (desiredMode_)
    {
        case eOutputMode::PWM:
            _channelInfo[channelID_].mode = eOutputMode::PWM;
            currentMode = this->eOutputModeToStr(_channelInfo[channelID_].mode);
            RCLCPP_INFO(this->get_logger(), "Current mode: %s", currentMode.c_str());
            break;

        case eOutputMode::FIX:
            _channelInfo[channelID_].mode = eOutputMode::FIX;
            currentMode = this->eOutputModeToStr(_channelInfo[channelID_].mode);
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
bool DDBControlNode::setPWMValues(float dutyCycle_, float frequency_, uint8_t channelID_)
{
    bool isUpdated = true;

    _channelInfo[channelID_].frequency = frequency_;
    RCLCPP_INFO(this->get_logger(), "Frequency set to %f", _channelInfo[channelID_].frequency);

    _channelInfo[channelID_].dutyCycle = dutyCycle_;
    RCLCPP_INFO(this->get_logger(), "Duty cycle set to %d", _channelInfo[channelID_].dutyCycle);

    return isUpdated;
}

/**
 * @brief Takes the current mode and changes it for a std::string
 *
 * @param mode_ Current mode of the channel
 * @return std::string corresponding to the mode asked
 */
std::string DDBControlNode::eOutputModeToStr(eOutputMode mode_)
{
    switch (mode_)
    {
        case eOutputMode::FIX:
            return "FIX";
        case eOutputMode::PWM:
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
void DDBControlNode::setStateLogic(const rover_msgs::srv::DDBControl::Request& request_,
                                   rover_msgs::srv::DDBControl::Response& response_)
{
    if (this->setChannelOutput(request_.channel_id, request_.output_state))
    {
        std::string msg = "Channel #" + std::to_string(request_.channel_id) + " set to " + std::to_string(request_.output_state);
        RCLCPP_INFO(this->get_logger(), msg.c_str());
        response_.success = true;
        response_.current_output_state = eOutputStateToUint8_t(_channelInfo[request_.channel_id].state);
        response_.status = msg;
    }
    else
    {
        std::string msg
            = "Could not set channel #" + std::to_string(request_.channel_id) + " to " + std::to_string(request_.output_state);
        RCLCPP_ERROR(this->get_logger(), msg.c_str());
        response_.success = false;
        response_.current_output_state = eOutputStateToUint8_t(_channelInfo[request_.channel_id].state);
        response_.status = msg;
    }
}

/**
 * @brief Main logic for switching mode of the desired channel. Only in bank 0
 *
 * @param request_
 * @param response_
 */
void DDBControlNode::setModeLogic(const rover_msgs::srv::DDBControl::Request& request_,
                                  rover_msgs::srv::DDBControl::Response& response_)
{
    eOutputMode desiredMode = eOutputMode::FIX;

    if (request_.output_state == rover_msgs::srv::DDBControl::Request::OUTPUT_ON
        || request_.output_state == rover_msgs::srv::DDBControl::Request::OUTPUT_OFF)
    {
        desiredMode = eOutputMode::FIX;
    }
    else if (request_.output_state == rover_msgs::srv::DDBControl::Request::OUTPUT_PWM)
    {
        desiredMode = eOutputMode::PWM;
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid mode for channel: %d", request_.channel_id);
        response_.success = false;
        response_.status = "Invalid mode for channel: " + std::to_string(request_.channel_id);
        return;
    }

    if (!setOutputMode(request_.channel_id, desiredMode))
    {
        std::string msg = "Mode could not be set to " + std::to_string(request_.output_state) + " for channel #"
                          + std::to_string(request_.channel_id);
        RCLCPP_ERROR(this->get_logger(), msg.c_str());
        response_.success = false;
        response_.current_output_state = eOutputStateToUint8_t(_channelInfo[request_.channel_id].state);
        response_.status = msg;
    }
    else
    {
        std::string msg = "Succesfully assigned mode " + eOutputModeToStr(_channelInfo[request_.channel_id].mode)
                          + " for channel #" + std::to_string(request_.channel_id);
        RCLCPP_INFO(this->get_logger(), "%s", msg.c_str());
        response_.success = true;
        response_.current_output_state = eOutputStateToUint8_t(_channelInfo[request_.channel_id].state);
        response_.status = msg;
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
bool DDBControlNode::valuesCheck(float dutyCycle_,
                                 float frequency_,
                                 uint8_t channelID_,
                                 rover_msgs::srv::DDBControl::Response& response_)
{
    if (dutyCycle_ > 100.0 || dutyCycle_ < 0.0)
    {
        std::string msg = "Duty cycle must be in range [0.0; 100.0]. Duty cycle received: " + std::to_string(dutyCycle_);
        RCLCPP_ERROR(this->get_logger(), msg.c_str());
        response_.success = false;
        response_.current_output_state = eOutputStateToUint8_t(_channelInfo[channelID_].state);
        response_.status = msg;
        return false;
    }

    if (frequency_ == 0.0F || frequency_ < 0.0F)
    {
        std::string msg = "Frequency must in range ]0.0; 100.0]. Frequency received: " + std::to_string(frequency_);
        RCLCPP_ERROR(this->get_logger(), msg.c_str());
        response_.success = false;
        response_.current_output_state = eOutputStateToUint8_t(_channelInfo[channelID_].state);
        response_.status = msg;
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
void DDBControlNode::setValuesLogic(const rover_msgs::srv::DDBControl::Request& request_,
                                    rover_msgs::srv::DDBControl::Response& response_)
{
    switch (_channelInfo[request_.channel_id].mode)
    {
        case eOutputMode::FIX:
            RCLCPP_ERROR(this->get_logger(), "Current mode doesn't allow changing PWM values");
            break;

        case eOutputMode::PWM:
            if (this->setPWMValues(request_.duty_cycle, request_.frequency, request_.channel_id))
            {
                std::string msg = "PWM values changed for channel #" + std::to_string(request_.channel_id) + "\nDuty cycle: "
                                  + std::to_string(request_.duty_cycle) + "\nFrequency: " + std::to_string(request_.frequency);
                RCLCPP_INFO(this->get_logger(), msg.c_str());
                response_.success = true;
                response_.current_output_state = eOutputStateToUint8_t(_channelInfo[request_.channel_id].state);
                response_.status = msg;
            }
            else
            {
                std::string msg = "PWM values could not be changed for channel #" + std::to_string(request_.channel_id)
                                  + ". Check logs for details.";
                RCLCPP_ERROR(this->get_logger(), msg.c_str());
                response_.success = false;
                response_.current_output_state = eOutputStateToUint8_t(_channelInfo[request_.channel_id].state);
                response_.status = msg;
            }
            break;
    }
}

uint8_t DDBControlNode::eOutputStateToUint8_t(eOutputState state_)
{
    if (state_ == eOutputState::ON)
    {
        return rover_msgs::srv::DDBControl::Request::OUTPUT_ON;
    }
    else
    {
        return rover_msgs::srv::DDBControl::Request::OUTPUT_OFF;
    }
}