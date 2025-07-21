#include "antenna_node.hpp"
#include <rover_lib2/helpers/constants.hpp>

namespace
{
    rover_msgs::msg::AntennaStatus toRosMsg(const sAntennaMsg& msg_)
    {
        rover_msgs::msg::AntennaStatus rosMsg;
        rosMsg.connected = msg_.connected;
        rosMsg.rssi = msg_.rssi;
        rosMsg.rxrate = msg_.rxRate;
        rosMsg.txrate = msg_.txRate;
        return rosMsg;
    }
}  // namespace

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<AntennaNode>());

    rclcpp::shutdown();
    return 0;
}

AntennaNode::AntennaNode():
    rclcpp::Node("antenna"),
    _driver(PUBLISHER_PERIOD_MS)
{
    _pub_antennaStatus = this->create_publisher<rover_msgs::msg::AntennaStatus>(TOPIC_ANTENNA_STATUS, QOS_DEFAULT);

    if (!loadUserInfo())
    {
        RCLCPP_ERROR(this->get_logger(), "Antenna credentials where not found in ENV, check your baschrc");
    }
    else
    {
        _driver.setUserInfo(_username, _password);
        _timer_pub = this->create_wall_timer(std::chrono::milliseconds(PUBLISHER_PERIOD_MS),
                                             [this](void)
                                             {
                                                 this->executeDriver();
                                             });
    }
}

bool AntennaNode::loadUserInfo(void)
{
    const char* usernameEnv = std::getenv("ROVER_USERNAME");
    const char* passwordEnv = std::getenv("ROVER_PASSWORD");

    if (!usernameEnv || !passwordEnv)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "ROVER_USERNAME or ROVER_PASSWORD environment variable not set, see bashrc setup in rover doc");
        return false;
    }

    _username = usernameEnv;
    _password = passwordEnv;
    RCLCPP_DEBUG(this->get_logger(), "Loaded username and password from environment variables");
    return true;
}

void AntennaNode::executeDriver(void)
{
    sAntennaMsg msg;
    sCommandResult result;
    result = _driver.ExecuteAntennaCommands(msg);

    rover_msgs::msg::AntennaStatus rosMsg = toRosMsg(msg);
    _pub_antennaStatus->publish(rosMsg);

    if (!result.error.empty())
    {
        if (result.success)
        {
            RCLCPP_INFO(this->get_logger(), "%s", result.error.c_str());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "%s", result.error.c_str());
        }
    }
}