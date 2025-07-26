#include "antenna_node.hpp"
#include <rover_lib2/helpers/constants.hpp>

namespace
{
    rover_msgs::msg::AntennaStatus toRosMsg(const sSignalInfos& msg_)
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
    rclcpp::Node("antenna")
{
    _pub_antennaStatus = this->create_publisher<rover_msgs::msg::AntennaStatus>(TOPIC_ANTENNA_STATUS, QOS_DEFAULT);

    if (!this->loadUserInfo())
    {
        RCLCPP_ERROR(this->get_logger(), "Antenna credentials where not found in ENV, check your baschrc");
    }
    else
    {
        _driver = std::make_unique<AntennaDriver>(PUBLISHER_PERIOD_MS, _username, _password);
        _timer_pubAntennaStatus = this->create_wall_timer(std::chrono::milliseconds(PUBLISHER_PERIOD_MS),
                                                          [this](void)
                                                          {
                                                              this->retrieveDriverInfosAndPublish();
                                                          });
    }
}

bool AntennaNode::loadUserInfo(void)
{
    const char* usernameEnv = std::getenv(ANTENNA_BASE_USERNAME);
    const char* passwordEnv = std::getenv(ANTENNA_BASE_PASSWORD);

    if (!usernameEnv || !passwordEnv)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "ROVER_ANTENNA_BASE_USERNAME or ROVER_ANTENNA_BASE_PASSWORD environment variable not set, see bashrc setup "
                     "in rover doc");
        return false;
    }

    _username = usernameEnv;
    _password = passwordEnv;
    RCLCPP_DEBUG(this->get_logger(), "Loaded username and password from environment variables");
    return true;
}

void AntennaNode::retrieveDriverInfosAndPublish(void)
{
    sSignalInfos msg;
    eAntennaCode result;
    result = _driver->retrieveDatalinkInfos(msg);

    rover_msgs::msg::AntennaStatus rosMsg = toRosMsg(msg);
    _pub_antennaStatus->publish(rosMsg);

    switch (result)
    {
        case eAntennaCode::SUCCESS:
            /*No log on success*/
            break;
        case eAntennaCode::FAILURE_DEVICE_OFFLINE:
            RCLCPP_ERROR(this->get_logger(), "Base antenna is offline");
            break;
        case eAntennaCode::FAILURE_SESSION_EXPIRED:
            RCLCPP_ERROR(this->get_logger(), "Access forbidden, check antenna connection or your credentials in doc");
            break;
        case eAntennaCode::FAILURE_PARSING_ERROR:
            RCLCPP_ERROR(this->get_logger(),
                         "Parsing error, wrong return type, probable cause: incorrect credentials. Refer to Documentation");
            break;
        case eAntennaCode::FAILURE_ON_COOLDOWN:
            /*No log on cooldown*/
            break;
        case eAntennaCode::FAILURE_UNKNOWN:
            [[fallthrough]];
        default:
            RCLCPP_ERROR(this->get_logger(), "Couln't retrieve data link info, unknown error");
            break;
    }
}