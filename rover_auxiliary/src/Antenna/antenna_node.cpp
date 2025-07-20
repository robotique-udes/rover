#include "antenna_node.hpp"
#include <rover_lib2/helpers/constants.hpp>
#include <fstream>
#include <sstream>

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
        RCLCPP_INFO(this->get_logger(), loginResult.error.c_str());

        _timer_pub = this->create_wall_timer(std::chrono::milliseconds(PUBLISHER_PERIOD_MS),
                                             [this](void)
                                             {
                                                 sAntennaMsg msg;
                                                 sCommandResult result;
                                                 result = _driver.CbAntennaPublisher(msg);

                                                 rover_msgs::msg::AntennaStatus rosMsg = toRosMsg(msg);
                                                 _pubAntennaStatus->publish(rosMsg);

                                                 if (!result.error.empty())
                                                 {
                                                     if (result.success)
                                                     {
                                                         RCLCPP_INFO(this->get_logger(), result.error.c_str());
                                                     }
                                                     else
                                                     {
                                                         RCLCPP_ERROR(this->get_logger(), result.error.c_str());
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
    const char* home = std::getenv("HOME");
    std::string homeStr;
    if (home)
    {
        homeStr = home;
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Unable to locate HOME folder to load env variable in antenna");
        return false;
    }

    std::string filepath = homeStr + ENV_PATH;
    bool userFound = false;
    bool passwordFound = false;

    std::ifstream file(filepath);
    if (!file.is_open())
    {
        RCLCPP_WARN(this->get_logger(), "Could not open .env file: %s", filepath.c_str());
        return false;
    }

    std::string line;
    while (std::getline(file, line))
    {
        if (line.empty() || line[0] == '#')
        {
            continue;
        }

        size_t pos = line.find('=');
        if (pos == std::string::npos)
        {
            continue;
        }

        std::string key = line.substr(0, pos);
        std::string value = line.substr(pos + 1);

        if (value.length() >= 2 && value.front() == '"' && value.back() == '"')
        {
            value = value.substr(1, value.length() - 2);
        }

        if (key == "username")
        {
            _username = value;
            userFound = true;
            RCLCPP_DEBUG(this->get_logger(), "Loaded username from .env");
        }
        else if (key == "password")
        {
            _password = value;
            passwordFound = true;
            RCLCPP_DEBUG(this->get_logger(), "Loaded password from .env");
        }
    }

    file.close();
    return passwordFound && userFound;
}