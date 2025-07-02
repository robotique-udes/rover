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

// Add after constructor but before CB_antenna_publisher

bool AntennaNode::login()
{
    if (_is_logged_in) {
        return true;
    }

    if (!_session) {
        _session = std::make_shared<cpr::Session>();
    }

    // Configure session with SSL settings
    _session->SetVerifySsl(false);
    
    cpr::SslOptions ssl_options;
    ssl_options.ciphers = "DEFAULT@SECLEVEL=1";
    ssl_options.verify_peer = false;
    ssl_options.verify_host = false;
    _session->SetOption(ssl_options);
    
    // Set timeouts
    _session->SetConnectTimeout(cpr::ConnectTimeout{5000});
    _session->SetTimeout(cpr::Timeout{10000});
    
    // Set the login URL
    _session->SetUrl(cpr::Url{Constants::AntennaInfo::ANTENNA_URL_MAP.at("Base") + "/login.cgi"});
    
    // Create the login payload
    cpr::Payload payload{
        {"username", username},
        {"password", password}
    };

    _session->SetOption(payload);
    
    RCLCPP_INFO(this->get_logger(), "Attempting to login to antenna...");
    
    // Send the login POST request
    cpr::Response response = _session->Post();
    
    if (response.status_code == 200) {
        _is_logged_in = true;
        RCLCPP_INFO(this->get_logger(), "Antenna login successful");
        return true;
    } else {
        RCLCPP_ERROR(this->get_logger(), "Antenna login failed: %s (code: %ld)", 
                    response.error.message.c_str(), response.status_code);
        return false;
    }
}

void AntennaNode::CB_antenna_publisher(void)
{
    rover_msgs::msg::AntennaStatus msg;
    if (Constants::AntennaInfo::ANTENNA_URL_MAP.find("Base") == Constants::AntennaInfo::ANTENNA_URL_MAP.end())
    {
        msg.success = false;
        msg.status = "Couldn't find the Base antenna URL in the URL map";
        _pub_antenna_status->publish(msg);
        RCLCPP_ERROR(this->get_logger(), "HERE HERE");
        return;
    }

    // Try logging in if not already logged in
    if (!_is_logged_in && !login()) {
        msg.success = false;
        msg.status = "Failed to login to antenna";
        msg.http_code = 0;
        _pub_antenna_status->publish(msg);
        return;
    }

    // Now use the existing session with stored cookies for the status request
    _session->SetUrl(cpr::Url{Constants::AntennaInfo::ANTENNA_URL_MAP.at("Base") + "/status.cgi"});
    
    // Debug callback can be retained
    _session->SetDebugCallback(cpr::DebugCallback(
        [this](cpr::DebugCallback::InfoType type, std::string data, intptr_t /*userdata*/)
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

    // Send the GET request using the same session (which has the cookies)
    _session->SetOption(cpr::Payload{});
    cpr::Response response = _session->Get();
    
    // Check if our session expired
    if (response.status_code == 401 || response.status_code == 403) {
        RCLCPP_WARN(this->get_logger(), "Session appears expired, attempting to re-login");
        _is_logged_in = false;
        
        if (login()) {
            // Retry the request with fresh session
            _session->SetUrl(cpr::Url{Constants::AntennaInfo::ANTENNA_URL_MAP.at("Base") + "/status.cgi"});
            response = _session->Get();
        }
    }

    // Populate and publish the message
    msg.status = response.error.message;
    msg.raw_json = response.text;
    msg.http_code = response.status_code;
    msg.success = (response.status_code >= 200 && response.status_code < 300);

    _pub_antenna_status->publish(msg);
}