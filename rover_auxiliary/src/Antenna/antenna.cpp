#include "antenna.hpp"
#include <iostream>
#include <charconv>
#include <rover_lib2/helpers/constants.hpp>
#include <json/json.h>

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
    if (_is_logged_in)
    {
        return true;
    }

    if (!_session)
    {
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
    cpr::Payload payload{{"username", username}, {"password", password}};

    _session->SetOption(payload);

    RCLCPP_INFO(this->get_logger(), "Attempting to login to antenna...");

    // Send the login POST request
    cpr::Response response = _session->Post();

    if (response.status_code == 200)
    {
        _is_logged_in = true;
        RCLCPP_INFO(this->get_logger(), "Antenna login successful");
        return true;
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Antenna login failed: %s (code: %ld)",
                     response.error.message.c_str(),
                     response.status_code);
        return false;
    }
}

void AntennaNode::CB_antenna_publisher(void)
{
    rover_msgs::msg::AntennaStatus msg;
    if (this->getStatus(&msg) && this->getIfStats(&msg))
    {
        msg.success = 1;
    }
    else
    {
        msg.success = 0;
    }
    _pub_antenna_status->publish(msg);
}

bool AntennaNode::getIfStats(rover_msgs::msg::AntennaStatus* msg_)
{
    if (!msg_)
    {
        return false;
    }

    if (Constants::AntennaInfo::ANTENNA_URL_MAP.find("Base") == Constants::AntennaInfo::ANTENNA_URL_MAP.end())
    {
        msg_->success = false;
        msg_->status = "Couldn't find the Base antenna URL in the URL map";
        return false;
    }

    // Try logging in if not already logged in
    if (!_is_logged_in && !login())
    {
        msg_->success = false;
        msg_->status = "Failed to login to antenna";
        msg_->http_code = 0;
        return false;
    }

    // Now use the existing session with stored cookies for the status request
    _session->SetUrl(cpr::Url{Constants::AntennaInfo::ANTENNA_URL_MAP.at("Base") + "/ifstats.cgi"});

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
    if (response.status_code == 401 || response.status_code == 403)
    {
        RCLCPP_WARN(this->get_logger(), "Session appears expired, attempting to re-login");
        _is_logged_in = false;

        if (login())
        {
            // Retry the request with fresh session
            _session->SetUrl(cpr::Url{Constants::AntennaInfo::ANTENNA_URL_MAP.at("Base") + "/status.cgi"});
            response = _session->Get();
        }
    }

    // Populate and publish the message
    msg_->status = response.error.message;
    msg_->raw_json = response.text;
    msg_->http_code = response.status_code;

    if (!(response.status_code >= 200 && response.status_code < 300))
    {
        return false;
    }

    if (!(response.status_code >= 200 && response.status_code < 300))
    {
        return false;
    }
    else if (!response.text.empty())
    {
        Json::Value root;
        Json::CharReaderBuilder builder;
        std::string errors;

        std::istringstream stream(response.text);
        if (Json::parseFromStream(builder, stream, &root, &errors))
        {
            if (root.isMember("interfaces") && root["interfaces"].isArray())
            {
                Json::Value interfaces = root["interfaces"];

                // Look for the first interface (index 0)
                if (interfaces.size() > 0 && interfaces[0].isObject())
                {
                    Json::Value interface0 = interfaces[0];

                    if (interface0.isMember("stats") && interface0["stats"].isObject())
                    {
                        Json::Value stats = interface0["stats"];

                        if (stats.isMember("rx_bytes"))
                        {
                            std::string wlanRxBytesStr = stats["rx_bytes"].asString();
                            uint64_t wlanRxBytes;
                            std::from_chars_result result = std::from_chars(wlanRxBytesStr.data(), wlanRxBytesStr.data() + wlanRxBytesStr.size(), wlanRxBytes);
                            
                            if (result.ec == std::errc{})
                            {
                                float wlanRxRate = (wlanRxBytes - _wlanRxBytes) / PUBLISHER_PERIOD_MS;
                                msg_->wlan_rxrate = wlanRxRate;
                                _wlanRxBytes = wlanRxBytes;
                                RCLCPP_INFO(this->get_logger(), "wlan Rx Rate: %f", wlanRxRate);
                            }
                            else
                            {
                                RCLCPP_ERROR(this->get_logger(), "Failed to parse rx_bytes: %s", wlanRxBytesStr.c_str());
                            }
                        }

                        if (stats.isMember("tx_bytes"))
                        {
                            std::string wlanTxBytesStr = stats["tx_bytes"].asString();
                            uint64_t wlanTxBytes;
                            std::from_chars_result result = std::from_chars(wlanTxBytesStr.data(), wlanTxBytesStr.data() + wlanTxBytesStr.size(), wlanTxBytes);
                            
                            if (result.ec == std::errc{})
                            {
                                float wlanTxRate = (wlanTxBytes - _wlanTxBytes) / PUBLISHER_PERIOD_MS;
                                msg_->wlan_txrate = wlanTxRate;
                                _wlanTxBytes = wlanTxBytes;
                                RCLCPP_INFO(this->get_logger(), "wlan Tx Rate: %f", wlanTxRate);
                            }
                            else
                            {
                                RCLCPP_ERROR(this->get_logger(), "Failed to parse rx_bytes: %s", wlanTxBytesStr.c_str());
                            }
                        }
                    }
                }

                if (interfaces.size() > 0 && interfaces[1].isObject())
                {
                    Json::Value interface1 = interfaces[1];

                    if (interface1.isMember("stats") && interface1["stats"].isObject())
                    {
                        Json::Value stats = interface1["stats"];

                        if (stats.isMember("rx_bytes"))
                        {
                            std::string lanRxBytesStr = stats["rx_bytes"].asString();
                            uint64_t lanRxBytes;
                            std::from_chars_result result = std::from_chars(lanRxBytesStr.data(), lanRxBytesStr.data() + lanRxBytesStr.size(), lanRxBytes);
                            
                            if (result.ec == std::errc{})
                            {
                                float lanRxRate = (lanRxBytes - _lanRxBytes) / PUBLISHER_PERIOD_MS;
                                msg_->lan_rxrate = lanRxRate;
                                _wlanRxBytes = lanRxBytes;
                                RCLCPP_INFO(this->get_logger(), "lan Rx Rate: %f", lanRxRate);
                            }
                            else
                            {
                                RCLCPP_ERROR(this->get_logger(), "Failed to parse rx_bytes: %s", lanRxBytesStr.c_str());
                            }
                        }

                        if (stats.isMember("tx_bytes"))
                        {
                            std::string lanTxBytesStr = stats["tx_bytes"].asString();
                            uint64_t lanTxBytes;
                            std::from_chars_result result = std::from_chars(lanTxBytesStr.data(), lanTxBytesStr.data() + lanTxBytesStr.size(), lanTxBytes);
                            
                            if (result.ec == std::errc{})
                            {
                                float lanTxRate = (lanTxBytes - _lanTxBytes) / PUBLISHER_PERIOD_MS;
                                msg_->lan_txrate = lanTxRate;
                                _lanTxBytes = lanTxBytes;
                                RCLCPP_INFO(this->get_logger(), "lan Tx Rate: %f ", lanTxRate);
                            }
                            else
                            {
                                RCLCPP_ERROR(this->get_logger(), "Failed to parse rx_bytes: %s", lanTxBytesStr.c_str());
                            }
                        }
                    }
                }
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse JSON: %s", errors.c_str());
            return false;
        }
    }
    return true;
}

bool AntennaNode::getStatus(rover_msgs::msg::AntennaStatus* msg_)
{
    if (!msg_)
    {
        return false;
    }

    if (Constants::AntennaInfo::ANTENNA_URL_MAP.find("Base") == Constants::AntennaInfo::ANTENNA_URL_MAP.end())
    {
        msg_->success = false;
        msg_->status = "Couldn't find the Base antenna URL in the URL map";
        return false;
    }

    // Try logging in if not already logged in
    if (!_is_logged_in && !login())
    {
        msg_->success = false;
        msg_->status = "Failed to login to antenna";
        msg_->http_code = 0;
        return false;
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
    if (response.status_code == 401 || response.status_code == 403)
    {
        RCLCPP_WARN(this->get_logger(), "Session appears expired, attempting to re-login");
        _is_logged_in = false;

        if (login())
        {
            // Retry the request with fresh session
            _session->SetUrl(cpr::Url{Constants::AntennaInfo::ANTENNA_URL_MAP.at("Base") + "/status.cgi"});
            response = _session->Get();
        }
    }

    // Populate and publish the message
    msg_->status = response.error.message;
    msg_->raw_json = response.text;
    msg_->http_code = response.status_code;

    if (!(response.status_code >= 200 && response.status_code < 300))
    {
        return false;
    }
    else if (!response.text.empty())
    {
        Json::Value root;
        Json::CharReaderBuilder builder;
        std::string errors;

        std::istringstream stream(response.text);
        if (Json::parseFromStream(builder, stream, &root, &errors))
        {
            if (root.isMember("wireless"))
            {
                Json::Value wireless = root["wireless"];

                if (wireless.isMember("rssi"))
                {
                    int rssi = wireless["rssi"].asInt();
                    msg_->rssi = rssi;
                    RCLCPP_INFO(this->get_logger(), "rssi: %d", rssi);
                }

                if (wireless.isMember("txrate"))
                {
                    std::string txrate = wireless["txrate"].asString();
                    msg_->txrate = txrate;
                    RCLCPP_INFO(this->get_logger(), "txrate: %s", txrate.c_str());
                }

                if (wireless.isMember("rxrate"))
                {
                    std::string rxrate = wireless["rxrate"].asString();
                    msg_->rxrate = rxrate;
                    RCLCPP_INFO(this->get_logger(), "rxrate: %s", rxrate.c_str());
                }
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse JSON: %s", errors.c_str());
            return false;
        }
    }
    return true;
}