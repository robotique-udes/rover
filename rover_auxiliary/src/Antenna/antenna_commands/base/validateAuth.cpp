#include "validateAuth.hpp"

#include "getStatus.hpp"
#include <json/json.h>

Command::Base::ValidateAuth::ValidateAuth(const std::string baseURL_):
    AntennaCommand(baseURL_)
{
}

sCommandResult Command::Base::ValidateAuth::execute(std::shared_ptr<cpr::Session> session_, sAntennaMsg& msg_)
{
    cpr::Response response;
    sCommandResult result = this->getHTTPS(session_, response);
    result.httpStatus = response.status_code;
    if (!result)
    {
        msg_.clear();
        msg_.connected = false;
        return result;
    }

    result = this->validateFormat(response, msg_);

    if (!result)
    {
        msg_.clear();
        msg_.connected = false;
    }
    return result;
}

sCommandResult Command::Base::ValidateAuth::getHTTPS(std::shared_ptr<cpr::Session> session_, cpr::Response& response_)
{
    sCommandResult result;
    session_->SetUrl(cpr::Url{_baseURL + STATUS_PAGE});
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

        default:
            result.success = false;
            result.error = "Unexpected HTTP status using GET stats.cgi: " + std::to_string(response_.status_code);
            break;
    }
    return result;
}

sCommandResult Command::Base::ValidateAuth::validateFormat(const cpr::Response& response_, sAntennaMsg& msg_)
{
    sCommandResult result;
    if (response_.text.empty())
    {
        result.success = false;
        result.error = "Antenna response was empty, authentification invalid";
        return result;
    }

    Json::Value root;
    Json::CharReaderBuilder builder;
    std::string errors;
    std::istringstream stream(response_.text);

    if (Json::parseFromStream(builder, stream, &root, &errors) && root.isMember(JSON_FIELD_WIRELESS))
    {
        msg_.connected = true;
        result.success = true;
        return result;
    }
    else
    {
        result.success = false;
        result.error = "Antenna response was not valid JSON format, most likely cause: invalid credentials in .env";
        return result;
    }
}