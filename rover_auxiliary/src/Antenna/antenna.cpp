#include "antenna.hpp"
#include <iostream>
#include <rover_lib2/helpers/constants.hpp>

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
    _pub_antenna_status = this->create_publisher<rover_msgs::msg::AntennaStatus>(TOPIC_ANTENNA_STATUS, QOS_DEFAULT);

    _timer_pub = this->create_wall_timer(std::chrono::milliseconds(PUBLISHER_PERIOD_MS),
                                         [this]()
                                         {
                                             CB_antenna_publisher();
                                         });
}

AntennaNode::~AntennaNode() {}

void AntennaNode::CB_antenna_publisher(void)
{
    rover_msgs::msg::AntennaStatus msg;
    if (Constants::AntennaInfo::ANTENNA_URL_MAP.find("Base") == Constants::AntennaInfo::ANTENNA_URL_MAP.end())
    {
        msg.success = false;
        msg.status = "Couldn't find the Base antenna URL in the URL map";
        _pub_antenna_status->publish(msg);
        RCLCPP_ERROR(this->get_logger(), "HERE");
        return;
    }

    // Create a CPR session to configure the request
    cpr::Session session;

    // Set the URL
    session.SetUrl(cpr::Url{Constants::AntennaInfo::ANTENNA_URL_MAP.at("Base") + "/status.cgi"});

    // Set timeout for connection establishment (5 seconds)
    session.SetConnectTimeout(cpr::Timeout{5000});

    // Set timeout for the entire request (10 seconds)
    session.SetTimeout(cpr::Timeout{10000});

    // Disable SSL verification (equivalent to curl -k)
    session.SetVerifySsl(false);

    // Set specific SSL/TLS cipher options (equivalent to --ciphers DEFAULT@SECLEVEL=1)
    // This requires setting CURLOPT_SSL_CIPHER_LIST directly with CPR's option
    cpr::SslOptions ssl_options;
    ssl_options.ciphers = "DEFAULT@SECLEVEL=1";
    session.SetOption(ssl_options);

    // Instead of using cookie files, use authentication
    // Replace "username" and "password" with your actual credentials
    session.SetAuth(cpr::Authentication(username, password, cpr::AuthMode::BASIC));

    session.SetDebugCallback(cpr::DebugCallback(
        [this](cpr::DebugCallback::InfoType type, std::string data)
        {
            switch (type)
            {
                case cpr::DebugCallback::InfoType::TEXT:
                    RCLCPP_DEBUG(this->get_logger(), "HTTP Debug: %s", data.c_str());
                    break;
                case cpr::DebugCallback::InfoType::HEADER_IN:
                    RCLCPP_DEBUG(this->get_logger(), "HTTP Header In: %s", data.c_str());
                    break;
                case cpr::DebugCallback::InfoType::HEADER_OUT:
                    RCLCPP_DEBUG(this->get_logger(), "HTTP Header Out: %s", data.c_str());
                    break;
                case cpr::DebugCallback::InfoType::DATA_IN:
                    RCLCPP_DEBUG(this->get_logger(), "HTTP Data In: %zu bytes", data.size());
                    break;
                case cpr::DebugCallback::InfoType::DATA_OUT:
                    RCLCPP_DEBUG(this->get_logger(), "HTTP Data Out: %zu bytes", data.size());
                    break;
                case cpr::DebugCallback::InfoType::SSL_DATA_IN:
                case cpr::DebugCallback::InfoType::SSL_DATA_OUT:
                    // Usually too verbose
                    break;
            }
            return true;  // Return true to continue receiving debug info
        }));

    // Send the request
    cpr::Response response = session.Get();

    // Print the status code
    msg.status = response.error;
    msg.raw_json = response.text;
    msg.http_code = response.status_code;

    _pub_antenna_status->publish(msg);
}