#include "get_status.hpp"
#include <json/json.h>

Command::Base::GetStatus::GetStatus(const std::string& baseURL_):
    AntennaCommand(baseURL_)
{
}

sCommandResult Command::Base::GetStatus::execute(std::shared_ptr<cpr::Session> session_, sAntennaMsg& msg_)
{
    cpr::Response response;
    sCommandResult result = this->getHTTPS(session_, response);
    result.httpStatus = response.status_code;
    if (!result.success)
    {
        return result;
    }

    result = this->parseResponse(response, msg_);
    return result;
}

sCommandResult Command::Base::GetStatus::getHTTPS(std::shared_ptr<cpr::Session> session_, cpr::Response& response_)
{
    sCommandResult result;
    session_->SetUrl(cpr::Url{this->getApiUrl() + STATUS_PAGE});
    response_ = session_->Get();

    switch (response_.status_code)
    {
        case std::to_underlying(eHttpStatus::OK):
            result.success = true;
            break;

        case std::to_underlying(eHttpStatus::FORBIDDEN):
            [[fallthrough]];
        case std::to_underlying(eHttpStatus::UNAUTHORIZED):
            result.success = false;
            result.error = "Session expired";
            break;
        case std::to_underlying(eHttpStatus::OFFLINE):
            result.success = false;
            result.error = "Antenna is offline";
            break;
        default:
            result.success = false;
            result.error = "Unexpected HTTP status using GET stats.cgi: " + std::to_string(response_.status_code);
            break;
    }
    return result;
}

sCommandResult Command::Base::GetStatus::parseResponse(const cpr::Response& response_, sAntennaMsg& msg_)
{
    sCommandResult result;
    if (response_.text.empty())
    {
        result.success = false;
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
        result.success = true;
    }
    else
    {
        result.success = false;
        result.error
            = std::string("Failed to parse JSON: ") + errors + ", probable cause is invalid credentials, check your env variables";
    }
    return result;
}