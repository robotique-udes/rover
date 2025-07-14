#include "antenna_node.hpp"
#include <fstream>
#include <sstream>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<AntennaNode>());

    rclcpp::shutdown();
    return 0;
}

AntennaNode::AntennaNode():
    rclcpp::Node("antenna")
{
    _pubAntennaStatus = this->create_publisher<rover_msgs::msg::AntennaStatus>(TOPIC_ANTENNA_STATUS, QOS_DEFAULT);

    if (loadEnvFile())
    {
        _timer_pub = this->create_wall_timer(std::chrono::milliseconds(PUBLISHER_PERIOD_MS),
                                             [this]()
                                             {
                                                 CB_antenna_publisher();
                                             });
    }
    else
    {
        rover_msgs::msg::AntennaStatus msg;
        msg.status = "Error loading .env file";
        _pubAntennaStatus->publish(msg);
    }
}

bool AntennaDriver::loadEnvFile(void)
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
        // Skip empty lines and comments
        if (line.empty() || line[0] == '#')
        {
            continue;
        }

        // Find the '=' delimiter
        size_t pos = line.find('=');
        if (pos == std::string::npos)
        {
            continue;
        }

        std::string key = line.substr(0, pos);
        std::string value = line.substr(pos + 1);

        // Remove quotes if present
        if (value.length() >= 2 && value.front() == '"' && value.back() == '"')
        {
            value = value.substr(1, value.length() - 2);
        }

        // Set the appropriate member variables
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