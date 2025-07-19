#include "getStatus.hpp"
#include <json/json.h>

Command::GetStatus::GetStatus(const std::string baseURL_):
    _baseURL(baseURL_)
{
}

sCommandResult Command::GetStatus::execute(std::shared_ptr<cpr::Session> session_, sAntennaMsg& msg_)
{
    cpr::Response response;
    sCommandResult result = this->getHTTPS(session_, response);
    if (!result)
    {
        return result;
    }

    result = parseResponse(response, msg_);
    return result;
}

sCommandResult Command::GetStatus::getHTTPS(std::shared_ptr<cpr::Session> session_, cpr::Response& response)
{
    sCommandResult result;
    session_->SetUrl(cpr::Url{_baseURL + STATUS_PAGE});
    response = session_->Get();

    switch (response.status_code)
    {
        case std::to_underlying(eHttpStatus::OK):
            result.success = true;
            return result;
            break;

        case std::to_underlying(eHttpStatus::FORBIDDEN):
            [[fallthrough]];
        case std::to_underlying(eHttpStatus::UNAUTHORIZED):
            result.error = "Session expired";
            return result;
            break;
        default:
            result.error = "Unexpected HTTP status: " + std::to_string(response.status_code);
            return result;
            break;
    }
}

sCommandResult Command::GetStatus::parseResponse(const cpr::Response& response_, sAntennaMsg& msg_)
{
    sCommandResult result;
    if (response_.text.empty())
    {
        result.error = "Response was empty";
        return result;
    }

    Json::Value root;
    Json::CharReaderBuilder builder;
    std::string errors;

    std::istringstream stream(response_.text);
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
        result.error
            = std::string("Failed to parse JSON: ") + errors + ", probable cause is invalid credentials, check your .env file";
        return result;
    }
    result.success = true;
    return result;
}

/*
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
}*/