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
    _pubAntennaStatus = this->create_publisher<rover_msgs::msg::AntennaStatus>(TOPIC_ANTENNA_STATUS, QOS_DEFAULT);

    if (loadUserInfo())
    {
        sCommandResult loginResult = _driver.setUserInfo(_username, _password);
        RCLCPP_INFO(this->get_logger(),"%s", loginResult.error.c_str());

        _timer_pub = this->create_wall_timer(std::chrono::milliseconds(PUBLISHER_PERIOD_MS),
                                             [this](void)
                                             {
                                                 sAntennaMsg msg;
                                                 sCommandResult result;
                                                 result = _driver.ExecuteAntennaCommands(msg);

                                                 rover_msgs::msg::AntennaStatus rosMsg = toRosMsg(msg);
                                                 _pubAntennaStatus->publish(rosMsg);

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
                                             });
    }
    else
    {
        rover_msgs::msg::AntennaStatus msg;
        msg.connected = false;
        _pubAntennaStatus->publish(msg);
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