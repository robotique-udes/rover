#include "ddb_control_node.hpp"

#include <rover_lib2/helpers/macros.hpp>
#include <rover_lib2/helpers/constants.hpp>

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
    _srv_control_bank0 = this->create_service<rover_msgs::srv::DDBControl>(
        TOPIC_CONTROL_BANK_0,
        [this](const std::shared_ptr<rover_msgs::srv::DDBControl::Request> request_,
               std::shared_ptr<rover_msgs::srv::DDBControl::Response> response_)
        {
            if (!request_ || !response_)
            {
                RCLCPP_FATAL(this->get_logger(), "NULL request or response received.");
                return;
            }
            this->callbackDdbControlBank0(*request_, *response_);
        });

    _srv_control_bank1 = this->create_service<rover_msgs::srv::DDBControl>(
        TOPIC_CONTROL_BANK_1,
        [this](const std::shared_ptr<rover_msgs::srv::DDBControl::Request> request_,
               std::shared_ptr<rover_msgs::srv::DDBControl::Response> response_)
        {
            if (!request_ || !response_)
            {
                RCLCPP_FATAL(this->get_logger(), "NULL request or response received.");
                return;
            }
            this->callbackDdbControlBank1(*request_, *response_);
        });

    _pub_DDB_status = this->create_publisher<rover_msgs::msg::DDBControl>(TOPIC_DDB_STATE, QOS_DEFAULT);

    _timer_pub = this->create_wall_timer(std::chrono::milliseconds(DELAY_PUBLISHER_MS),
                                         [this](void)
                                         {
                                             this->callbackDdbStatus();
                                         });
}

void DDBControlNode::callbackDdbControlBank0(const rover_msgs::srv::DDBControl::Request& request_,
                                             rover_msgs::srv::DDBControl::Response& response_)
{
    if (request_.channel_id >= MAX_CHANNELS)
    {
        std::string msg = "Invalid channel, range is [0; " + std::to_string(MAX_CHANNELS - 1UL)
                          + "]. Received channel : " + std::to_string(request_.channel_id);
        RCLCPP_WARN(this->get_logger(), "%s", msg.c_str());
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
            this->setPWMValues(0.0, 0.0, request_.channel_id);
            break;

        case rover_msgs::srv::DDBControl::Request::OUTPUT_PWM:
            this->setStateLogic(request_, response_);

            if (valuesCheck(request_.duty_cycle, request_.frequency, request_.channel_id, response_))
            {
                this->setValuesLogic(request_, response_);
            }
            else
            {
                std::string msg = "Invalid frequency or duty cycle. Check logs for details.";
                RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
                response_.success = false;
                response_.current_output_state = TO_UNDERLYING(_channelInfo[request_.channel_id].state);
                response_.status = msg;
                return;
            }
            break;

        default:
            std::string msg = "Received request without a valid command" + std::to_string(request_.output_state);
            RCLCPP_WARN(this->get_logger(), "%s", msg.c_str());
            response_.success = false;
            response_.current_output_state = TO_UNDERLYING(_channelInfo[request_.channel_id].state);
            response_.status = msg;
    }
}

void DDBControlNode::callbackDdbControlBank1(const rover_msgs::srv::DDBControl::Request& request_,
                                             rover_msgs::srv::DDBControl::Response& response_)
{
    if (request_.channel_id >= MAX_CHANNELS)
    {
        std::string msg = "Invalid channel, range is [0; " + std::to_string(MAX_CHANNELS - 1UL)
                          + "]. Received channel : " + std::to_string(request_.channel_id);
        RCLCPP_WARN(this->get_logger(), "%s", msg.c_str());
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
        response_.current_output_state = TO_UNDERLYING(_channelInfo[request_.channel_id].state);
        response_.status = "Received request without a valid command: " + std::to_string(request_.output_state);
    }
}

bool DDBControlNode::setChannelOutput(uint8_t channelID_, eOutputState desiredState_)
{
    switch (desiredState_)
    {
        case eOutputState::ON:
            _channelInfo[channelID_].state = eOutputState::ON;
            RCLCPP_DEBUG(this->get_logger(), "Set channel #%d output as ON", channelID_);
            break;

        case eOutputState::OFF:
            _channelInfo[channelID_].state = eOutputState::OFF;
            RCLCPP_DEBUG(this->get_logger(), "Set channel #%d output as OFF", channelID_);
            break;

        case eOutputState::PWM:
            _channelInfo[channelID_].state = eOutputState::PWM;
            RCLCPP_DEBUG(this->get_logger(), "Set channel #%d output as PWM", channelID_);
            break;
    }
    return true;
}

bool DDBControlNode::setPWMValues(float dutyCycle_, float frequency_, uint8_t channelID_)
{
    bool isUpdated = true;

    _channelInfo[channelID_].frequency = frequency_;
    RCLCPP_INFO(this->get_logger(), "Frequency set to %f", _channelInfo[channelID_].frequency);

    _channelInfo[channelID_].dutyCycle = dutyCycle_;
    RCLCPP_INFO(this->get_logger(), "Duty cycle set to %f", _channelInfo[channelID_].dutyCycle);

    return isUpdated;
}

/**
 * @brief Main logic for changing state of the desired channel
 *
 * @param request_
 * @param response_
 */
void DDBControlNode::setStateLogic(const rover_msgs::srv::DDBControl::Request& request_,
                                   rover_msgs::srv::DDBControl::Response& response_)
{
    eOutputState wantedState = static_cast<eOutputState>(request_.output_state);

    if (this->setChannelOutput(request_.channel_id, wantedState))
    {
        std::string msg = "Channel #" + std::to_string(request_.channel_id) + " set to " + std::to_string(request_.output_state);
        RCLCPP_DEBUG(this->get_logger(), "%s", msg.c_str());
        response_.success = true;
        response_.current_output_state = TO_UNDERLYING(_channelInfo[request_.channel_id].state);
        response_.status = msg;
    }
    else
    {
        std::string msg
            = "Could not set channel #" + std::to_string(request_.channel_id) + " to " + std::to_string(request_.output_state);
        RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
        response_.success = false;
        response_.current_output_state = TO_UNDERLYING(_channelInfo[request_.channel_id].state);
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
    if (dutyCycle_ > 100.0F || dutyCycle_ < 0.0F)
    {
        std::string msg = "Duty cycle must be in range [0.0; 100.0]. Duty cycle received: " + std::to_string(dutyCycle_);
        RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
        response_.success = false;
        response_.current_output_state = TO_UNDERLYING(_channelInfo[channelID_].state);
        response_.status = msg;
        return false;
    }

    if (frequency_ == 0.0F || frequency_ < 0.0F)
    {
        std::string msg = "Frequency must in range ]0.0; 100.0]. Frequency received: " + std::to_string(frequency_);
        RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
        response_.success = false;
        response_.current_output_state = TO_UNDERLYING(_channelInfo[channelID_].state);
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
    if (_channelInfo[request_.channel_id].state == eOutputState::PWM)
    {
        if (this->setPWMValues(request_.duty_cycle, request_.frequency, request_.channel_id))
        {
            std::string msg = "PWM values changed for channel #" + std::to_string(request_.channel_id) + "\nDuty cycle: "
                              + std::to_string(request_.duty_cycle) + "\nFrequency: " + std::to_string(request_.frequency);
            RCLCPP_DEBUG(this->get_logger(), "%s", msg.c_str());
            response_.success = true;
            response_.current_output_state = TO_UNDERLYING(_channelInfo[request_.channel_id].state);
            response_.status = msg;
        }
        else
        {
            std::string msg = "PWM values could not be changed for channel #" + std::to_string(request_.channel_id)
                              + ". Check logs for details.";
            RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
            response_.success = false;
            response_.current_output_state = TO_UNDERLYING(_channelInfo[request_.channel_id].state);
            response_.status = msg;
        }
    }
}

void DDBControlNode::callbackDdbStatus(void)
{
    rover_msgs::msg::DDBControl msg;

    msg.bank0_ch0_onstate = TO_UNDERLYING(_channelInfo[0].state);
    msg.bank0_ch0_duty = _channelInfo[0].dutyCycle;
    msg.bank0_ch0_freq = _channelInfo[0].frequency;

    msg.bank0_ch1_onstate = TO_UNDERLYING(_channelInfo[1].state);
    msg.bank0_ch1_duty = _channelInfo[1].dutyCycle;
    msg.bank0_ch1_freq = _channelInfo[1].frequency;

    msg.bank0_ch2_onstate = TO_UNDERLYING(_channelInfo[2].state);
    msg.bank0_ch2_duty = _channelInfo[2].dutyCycle;
    msg.bank0_ch2_freq = _channelInfo[2].frequency;

    msg.bank0_ch3_onstate = TO_UNDERLYING(_channelInfo[3].state);
    msg.bank0_ch3_duty = _channelInfo[3].dutyCycle;
    msg.bank0_ch3_freq = _channelInfo[3].frequency;

    msg.bank1_ch0_onstate = TO_UNDERLYING(_channelInfo2[0]);
    msg.bank1_ch1_onstate = TO_UNDERLYING(_channelInfo2[1]);
    msg.bank1_ch2_onstate = TO_UNDERLYING(_channelInfo2[2]);
    msg.bank1_ch3_onstate = TO_UNDERLYING(_channelInfo2[3]);

    _pub_DDB_status->publish(msg);
}

bool DDBControlNode::setChannelOutput2(uint8_t channelID_, eOutputState desiredState_)
{
    switch (desiredState_)
    {
        case eOutputState::ON:
            _channelInfo2[channelID_] = eOutputState::ON;
            RCLCPP_DEBUG(this->get_logger(), "Set channel #%d output as ON", channelID_);
            break;

        case eOutputState::OFF:
            _channelInfo2[channelID_] = eOutputState::OFF;
            RCLCPP_DEBUG(this->get_logger(), "Set channel #%d output as OFF", channelID_);
            break;

        case eOutputState::PWM:
            _channelInfo2[channelID_] = eOutputState::PWM;
            RCLCPP_DEBUG(this->get_logger(), "Set channel #%d output as PWM", channelID_);
            break;
    }
    return true;
}

void DDBControlNode::setStateLogic2(const rover_msgs::srv::DDBControl::Request& request_,
                                    rover_msgs::srv::DDBControl::Response& response_)
{
    eOutputState wantedState = static_cast<eOutputState>(request_.output_state);

    if (this->setChannelOutput2(request_.channel_id, wantedState))
    {
        std::string msg = "Channel #" + std::to_string(request_.channel_id) + " set to " + std::to_string(request_.output_state);
        RCLCPP_DEBUG(this->get_logger(), "%s", msg.c_str());
        response_.success = true;
        response_.current_output_state = TO_UNDERLYING(_channelInfo[request_.channel_id].state);
        response_.status = msg;
    }
    else
    {
        std::string msg
            = "Could not set channel #" + std::to_string(request_.channel_id) + " to " + std::to_string(request_.output_state);
        RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
        response_.success = false;
        response_.current_output_state = TO_UNDERLYING(_channelInfo[request_.channel_id].state);
        response_.status = msg;
    }
}