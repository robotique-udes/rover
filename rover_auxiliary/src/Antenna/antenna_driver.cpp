#include "antenna_driver.hpp"
#include <iostream>
#include <charconv>
#include <rover_lib2/helpers/constants.hpp>
#include <json/json.h>

AntennaDriver::AntennaDriver(const rclcpp::Logger& logger_, uint64_t publisherPeriodMs_) : _logger(logger_), _publisherPeriodMs(publisherPeriodMs_)
{
    _session = std::make_shared<cpr::Session>();
    this->setDebugCB();
}

void AntennaDriver::setUser(const std::string& username_, const std::string& password_)
{
    _username = username_;
    _password = password_;
}

bool AntennaDriver::login(void)
{
    if (_isLoggedIn)
    {
        return true;
    }

    if (_loginAttempts > MAX_LOGIN_ATTEMPTS)
    {
        RCLCPP_ERROR(_logger, "Max login attempts reached");
        return false;
    }
    _loginAttempts++;

    // Configure session with SSL settings
    _session->SetVerifySsl(false);

    cpr::SslOptions ssl_options;
    ssl_options.ciphers = HTTP_CIPHER;
    ssl_options.verify_peer = false;
    ssl_options.verify_host = false;
    _session->SetOption(ssl_options);

    // Set timeouts
    _session->SetConnectTimeout(cpr::ConnectTimeout{500});
    _session->SetTimeout(cpr::Timeout{1000});

    // Set the login URL
    _session->SetUrl(cpr::Url{Constants::AntennaInfo::ANTENNA_URL_MAP.at("Base") + LOGIN_PAGE});

    // Create the login payload
    cpr::Payload payload{{"username", _username}, {"password", _password}};

    _session->SetOption(payload);

    RCLCPP_INFO(_logger, "Attempting to login to antenna...");

    // Send the login POST request
    cpr::Response response = _session->Post();

    if (response.status_code == HTTP_SUCCESS_MIN)
    {
        _isLoggedIn = true;
        RCLCPP_INFO(_logger, "Post was successful");
        return true;
    }
    else
    {
        RCLCPP_ERROR(_logger,
                     "AntennaDriver login failed: %s (code: %ld)",
                     response.error.message.c_str(),
                     response.status_code);
        return false;
    }
}

void AntennaDriver::CbAntennaPublisher(rover_msgs::msg::AntennaStatus& msg_)
{
    if (_loginAttempts < MAX_LOGIN_ATTEMPTS && this->getStatus(msg_) && this->getIfStats(msg_))
    {
        msg_.success = true;
    }
    else
    {
        msg_.success = false;
    }
}

bool AntennaDriver::getIfStats(rover_msgs::msg::AntennaStatus& msg_)
{
    if (Constants::AntennaInfo::ANTENNA_URL_MAP.find("Base") == Constants::AntennaInfo::ANTENNA_URL_MAP.end())
    {
        msg_.success = false;
        msg_.status = "Couldn't find the Base antenna URL in the URL map";
        return false;
    }

    // Try logging in if not already logged in
    if (!_isLoggedIn && !login() && _loginAttempts < MAX_LOGIN_ATTEMPTS)
    {
        msg_.success = false;
        msg_.status = "Failed to login to antenna";
        msg_.http_code = 0;
        return false;
    }

    // Now use the existing session with stored cookies for the status request
    _session->SetUrl(cpr::Url{Constants::AntennaInfo::ANTENNA_URL_MAP.at("Base") + "/ifstats.cgi"});

    _session->SetOption(cpr::Payload{});  // remove payload from login
    cpr::Response response = _session->Get();

    // Check if our session expired
    if (response.status_code == HTTP_UNAUTHORIZED || response.status_code == HTTP_FORBIDDEN)
    {
        RCLCPP_WARN(_logger, "Session appears expired, attempting to re-login");
        _isLoggedIn = false;

        if (login())
        {
            _session->SetUrl(cpr::Url{Constants::AntennaInfo::ANTENNA_URL_MAP.at("Base") + "/ifstats.cgi"});
            response = _session->Get();
        }
    }

    // Populate and publish the message
    msg_.status = response.error.message;
    msg_.http_code = response.status_code;

    if (!(response.status_code >= HTTP_SUCCESS_MIN && response.status_code < HTTP_SUCCESS_MAX))
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
                            std::from_chars_result result = std::from_chars(wlanRxBytesStr.data(),
                                                                            wlanRxBytesStr.data() + wlanRxBytesStr.size(),
                                                                            wlanRxBytes);

                            if (result.ec == std::errc{})
                            {
                                float wlanRxRate = (wlanRxBytes - _wlanRxBytes) * 1000.0f / _publisherPeriodMs;
                                msg_.wlan_rxrate = wlanRxRate;
                                _wlanRxBytes = wlanRxBytes;
                            }
                            else
                            {
                                RCLCPP_ERROR(_logger, "Failed to parse rx_bytes: %s", wlanRxBytesStr.c_str());
                            }
                        }

                        if (stats.isMember("tx_bytes"))
                        {
                            std::string wlanTxBytesStr = stats["tx_bytes"].asString();
                            uint64_t wlanTxBytes;
                            std::from_chars_result result = std::from_chars(wlanTxBytesStr.data(),
                                                                            wlanTxBytesStr.data() + wlanTxBytesStr.size(),
                                                                            wlanTxBytes);

                            if (result.ec == std::errc{})
                            {
                                float wlanTxRate = (wlanTxBytes - _wlanTxBytes) * 1000.0f / _publisherPeriodMs;
                                msg_.wlan_txrate = wlanTxRate;
                                _wlanTxBytes = wlanTxBytes;
                            }
                            else
                            {
                                RCLCPP_ERROR(_logger, "Failed to parse tx_bytes: %s", wlanTxBytesStr.c_str());
                            }
                        }
                    }
                }

                if (interfaces.size() > 1 && interfaces[1].isObject())
                {
                    Json::Value interface1 = interfaces[1];

                    if (interface1.isMember("stats") && interface1["stats"].isObject())
                    {
                        Json::Value stats = interface1["stats"];

                        if (stats.isMember("rx_bytes"))
                        {
                            std::string lanRxBytesStr = stats["rx_bytes"].asString();
                            uint64_t lanRxBytes;
                            std::from_chars_result result
                                = std::from_chars(lanRxBytesStr.data(), lanRxBytesStr.data() + lanRxBytesStr.size(), lanRxBytes);

                            if (result.ec == std::errc{})
                            {
                                float lanRxRate = (lanRxBytes - _lanRxBytes) * 1000.0f / _publisherPeriodMs;
                                msg_.lan_rxrate = lanRxRate;
                                _lanRxBytes = lanRxBytes;
                            }
                            else
                            {
                                RCLCPP_ERROR(_logger, "Failed to parse rx_bytes: %s", lanRxBytesStr.c_str());
                            }
                        }

                        if (stats.isMember("tx_bytes"))
                        {
                            std::string lanTxBytesStr = stats["tx_bytes"].asString();
                            uint64_t lanTxBytes;
                            std::from_chars_result result
                                = std::from_chars(lanTxBytesStr.data(), lanTxBytesStr.data() + lanTxBytesStr.size(), lanTxBytes);

                            if (result.ec == std::errc{})
                            {
                                float lanTxRate = (lanTxBytes - _lanTxBytes) * 1000.0f / _publisherPeriodMs;
                                msg_.lan_txrate = lanTxRate;
                                _lanTxBytes = lanTxBytes;
                            }
                            else
                            {
                                RCLCPP_ERROR(_logger, "Failed to parse tx_bytes: %s", lanTxBytesStr.c_str());
                            }
                        }
                    }
                }
            }
        }
        else
        {
            RCLCPP_ERROR(_logger, "Unable to parse JSON: %s", errors.c_str());
            return false;
        }
    }
    return true;
}

bool AntennaDriver::getStatus(rover_msgs::msg::AntennaStatus& msg_)
{
    if (Constants::AntennaInfo::ANTENNA_URL_MAP.find("Base") == Constants::AntennaInfo::ANTENNA_URL_MAP.end())
    {
        msg_.success = false;
        msg_.status = "Couldn't find the Base antenna URL in the URL map";
        return false;
    }

    if (!_isLoggedIn && !login() && _loginAttempts < MAX_LOGIN_ATTEMPTS)
    {
        msg_.success = false;
        msg_.status = "Failed to login to antenna";
        msg_.http_code = 0;
        return false;
    }

    // Now use the existing session with stored cookies for the status request
    _session->SetUrl(cpr::Url{Constants::AntennaInfo::ANTENNA_URL_MAP.at("Base") + "/status.cgi"});

    // Send the GET request using the same session (which has the cookies)
    _session->SetOption(cpr::Payload{});
    cpr::Response response = _session->Get();

    // Check if our session expired
    if (response.status_code == HTTP_UNAUTHORIZED || response.status_code == HTTP_FORBIDDEN)
    {
        RCLCPP_WARN(_logger, "Session appears expired, attempting to re-login");
        _isLoggedIn = false;

        if (login())
        {
            // Retry the request with fresh session
            _session->SetUrl(cpr::Url{Constants::AntennaInfo::ANTENNA_URL_MAP.at("Base") + "/status.cgi"});
            response = _session->Get();
        }
    }

    // Populate and publish the message
    msg_.status = response.error.message;
    msg_.http_code = response.status_code;

    if (!(response.status_code >= HTTP_SUCCESS_MIN && response.status_code < HTTP_SUCCESS_MAX))
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
                    msg_.rssi = rssi;
                }

                if (wireless.isMember("txrate"))
                {
                    std::string txrate = wireless["txrate"].asString();
                    msg_.txrate = txrate;
                }

                if (wireless.isMember("rxrate"))
                {
                    std::string rxrate = wireless["rxrate"].asString();
                    msg_.rxrate = rxrate;
                }
            }
        }
        else
        {
            RCLCPP_ERROR_ONCE(_logger,
                              "Failed to parse JSON: %s, probable cause is invalid credentials, check your .env file",
                              errors.c_str());
            return false;
        }
    }
    return true;
}

void AntennaDriver::setDebugCB(void)
{
    if (!_session)
    {
        RCLCPP_ERROR(_logger, "Cannot set debug callback: session is null");
        return;
    }

    _session->SetDebugCallback(cpr::DebugCallback(
        [this](cpr::DebugCallback::InfoType type, std::string data, intptr_t /*userdata*/)
        {
            switch (type)
            {
                case cpr::DebugCallback::InfoType::TEXT:
                    RCLCPP_DEBUG(_logger, "HTTP Debug: %s", data.c_str());
                    break;
                case cpr::DebugCallback::InfoType::HEADER_IN:
                    RCLCPP_DEBUG(_logger, "HTTP Header In: %s", data.c_str());
                    break;
                case cpr::DebugCallback::InfoType::HEADER_OUT:
                    RCLCPP_DEBUG(_logger, "HTTP Header Out: %s", data.c_str());
                    break;
                case cpr::DebugCallback::InfoType::DATA_IN:
                    RCLCPP_DEBUG(_logger, "HTTP Data In: %zu bytes", data.size());
                    break;
                case cpr::DebugCallback::InfoType::DATA_OUT:
                    RCLCPP_DEBUG(_logger, "HTTP Data Out: %zu bytes", data.size());
                    break;
                case cpr::DebugCallback::InfoType::SSL_DATA_IN:
                case cpr::DebugCallback::InfoType::SSL_DATA_OUT:
                    // Usually too verbose
                    break;
            }
            return true;  // Return true to continue receiving debug info
        }));
}