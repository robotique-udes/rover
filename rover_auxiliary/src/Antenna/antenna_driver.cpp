#include "antenna_driver.hpp"
#include <iostream>
#include <charconv>
#include <json/json.h>

AntennaDriver::AntennaDriver(const rclcpp::Logger& logger_, uint64_t publisherPeriodMs_):
    _logger(logger_),
    _publisherPeriodMs(publisherPeriodMs_)
{
    _session = std::make_shared<cpr::Session>();
    this->setupSession();
    this->setDebugCB();
}

void AntennaDriver::setUserInfo(const std::string& username_, const std::string& password_)
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

    if (_loginAttempts >= MAX_LOGIN_ATTEMPTS)
    {
        RCLCPP_ERROR(_logger, "Max login attempts reached");
        return false;
    }
    _loginAttempts++;

    _session->SetUrl(cpr::Url{std::string(BASE_URL) + LOGIN_PAGE});
    cpr::Payload payload{{"username", _username}, {"password", _password}};
    _session->SetOption(payload);
    RCLCPP_INFO(_logger, "Attempting to login to antenna...");
    cpr::Response response = _session->Post();

    if (response.status_code >= HTTP_SUCCESS_MIN && response.status_code < HTTP_SUCCESS_MAX)
    {
        if (this->verifyAuthentication())
        {
            _isLoggedIn = true;
            RCLCPP_INFO(_logger, "Login was successful");
            _loginAttempts = 0;
            return true;
        }
        else
        {
            _isLoggedIn = false;
            RCLCPP_INFO(_logger, "Login unsuccessful, antenna was reached but username/password was wrong");
            return false;
        }
    }
    else
    {
        _isLoggedIn = false;
        RCLCPP_ERROR(_logger,
                     "AntennaDriver login failed, POST request was unsuccesfull: %s (code: %ld)",
                     response.error.message.c_str(),
                     response.status_code);
        return false;
    }
}

void AntennaDriver::CbAntennaPublisher(rover_msgs::msg::AntennaStatus& msg_)
{
    // when status fails don't bother with ifStats it will fail too
    // status is checked first because it's the shortest
    if (this->getStatus(msg_))
    {
        this->getIfStats(msg_);
    }
}

bool AntennaDriver::getIfStats(rover_msgs::msg::AntennaStatus& msg_)
{
    if (!this->isLoggedIn() && !login())
    {
        msg_.connected = false;
        msg_.info = "Failed to login to antenna";
        msg_.http_code = 0;
        return false;
    }

    _session->SetUrl(cpr::Url{std::string(BASE_URL) + IFSTATS_PAGE});

    _session->SetOption(cpr::Payload{});  // remove payload from login
    cpr::Response response = _session->Get();

    // Check if our session expired
    if (response.status_code == HTTP_UNAUTHORIZED || response.status_code == HTTP_FORBIDDEN)
    {
        RCLCPP_WARN(_logger, "Session appears expired, attempting to re-login");
        _isLoggedIn = false;

        if (login())
        {
            _session->SetUrl(cpr::Url{std::string(BASE_URL)+ IFSTATS_PAGE});
            response = _session->Get();
        }
        else
        {
            msg_.connected = false;
            msg_.info = "Failed to login to antenna";
            return false;
        }
    }

    // Populate and publish the message
    msg_.info = response.error.message;
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
            if (root.isMember(JSON_FIELD_INTERFACES) && root[JSON_FIELD_INTERFACES].isArray())
            {
                Json::Value interfaces = root[JSON_FIELD_INTERFACES];

                if (interfaces.size() > 0 && interfaces[INTERFACE_WLAN_INDEX].isObject())
                {
                    Json::Value interface0 = interfaces[INTERFACE_WLAN_INDEX];

                    if (interface0.isMember(JSON_FIELD_STATS) && interface0[JSON_FIELD_STATS].isObject())
                    {
                        Json::Value stats = interface0[JSON_FIELD_STATS];

                        if (stats.isMember(JSON_FIELD_RX_BYTES))
                        {
                            std::string wlanRxBytesStr = stats[JSON_FIELD_RX_BYTES].asString();
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

                        if (stats.isMember(JSON_FIELD_TX_BYTES))
                        {
                            std::string wlanTxBytesStr = stats[JSON_FIELD_TX_BYTES].asString();
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

    if (!this->isLoggedIn() && !login())
    {
        msg_.connected = false;
        msg_.info = "Failed to login to antenna";
        msg_.http_code = 0;
        return false;
    }

    // Now use the existing session with stored cookies for the status request
    _session->SetUrl(cpr::Url{std::string(BASE_URL) + STATUS_PAGE});

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
            _session->SetUrl(cpr::Url{std::string(BASE_URL) + STATUS_PAGE});
            response = _session->Get();
        }
        else
        {
            msg_.connected = false;
            msg_.info = "Failed to login to antenna";
            return false;
        }
    }

    // Populate and publish the message
    msg_.info = response.error.message;
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
            if (root.isMember(JSON_FIELD_WIRELESS))
            {
                Json::Value wireless = root[JSON_FIELD_WIRELESS];

                if (wireless.isMember(JSON_FIELD_RSSI))
                {
                    float rssi = wireless[JSON_FIELD_RSSI].asFloat();
                    msg_.rssi = rssi;
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

void AntennaDriver::setupSession(void)
{
    // RocketM2 general settings
    _session->SetVerifySsl(false);

    cpr::SslOptions ssl_options;
    ssl_options.ciphers = HTTP_CIPHER;
    ssl_options.verify_peer = false;
    ssl_options.verify_host = false;
    _session->SetOption(ssl_options);

    _session->SetConnectTimeout(cpr::ConnectTimeout{SESSION_CONNECT_TIMEOUT_MS});
    _session->SetTimeout(cpr::Timeout{SESSION_TIMEOUT_MS});
}

bool AntennaDriver::isLoggedIn(void)
{
    return _isLoggedIn;
}

bool AntennaDriver::verifyAuthentication(void)
{
    _session->SetUrl(cpr::Url{std::string(BASE_URL) + STATUS_PAGE});
    _session->SetOption(cpr::Payload{});
    cpr::Response response = _session->Get();

    if (response.status_code >= HTTP_SUCCESS_MIN && response.status_code < HTTP_SUCCESS_MAX && !response.text.empty())
    {
        // To validate authentification try to parse the GET response,
        // with a valid authentification GET will return json whrereas an invalid one will return html
        Json::Value root;
        Json::CharReaderBuilder builder;
        std::string errors;
        std::istringstream stream(response.text);

        if (Json::parseFromStream(builder, stream, &root, &errors) && root.isMember("wireless"))
        {
            return true;
        }
    }

    return false;
}