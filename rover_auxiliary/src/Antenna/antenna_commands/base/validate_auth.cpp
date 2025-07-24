#include "validate_auth.hpp"
#include <json/json.h>

Command::Base::ValidateAuth::ValidateAuth(const std::string& apiUrl_):
    AntennaCommand(apiUrl_)
{
}

eAntennaCode Command::Base::ValidateAuth::execute(std::shared_ptr<cpr::Session> session_, sSignalInfos& msg_)
{
    cpr::Response response;
    eAntennaCode result = this->getHTTPS(session_, response);
    // std::cout << response.text << std::endl;
    if (result != eAntennaCode::SUCCESS)
    {
        msg_ = sSignalInfos{};
        msg_.connected = false;
        return result;
    }

    result = this->validateFormat(response, msg_);

    if (result != eAntennaCode::SUCCESS)
    {
        msg_ = sSignalInfos{};
        msg_.connected = false;
    }
    return result;
}

eAntennaCode Command::Base::ValidateAuth::getHTTPS(std::shared_ptr<cpr::Session> session_, cpr::Response& response_)
{
    session_->SetUrl(cpr::Url{this->getApiUrl() + STATUS_PAGE});
    response_ = session_->Get();

    switch (response_.status_code)
    {
        case std::to_underlying(eHttpStatus::OK):
            return eAntennaCode::SUCCESS;
            break;

        case std::to_underlying(eHttpStatus::FORBIDDEN):
            [[fallthrough]];
        case std::to_underlying(eHttpStatus::UNAUTHORIZED):
            return eAntennaCode::FAILURE_SESSION_EXPIRED;
            break;
        case std::to_underlying(eHttpStatus::OFFLINE):
            return eAntennaCode::FAILURE_DEVICE_OFFLINE;
            break;

        default:
            return eAntennaCode::FAILURE_UNKNOWN;
            break;
    }
}

eAntennaCode Command::Base::ValidateAuth::validateFormat(const cpr::Response& response_, sSignalInfos& msg_)
{
    if (response_.text.empty())
    {
        return eAntennaCode::FAILURE_PARSING_ERROR;
    }

    Json::Value root;
    Json::CharReaderBuilder builder;
    std::string errors;
    std::istringstream stream(response_.text);

    if (Json::parseFromStream(builder, stream, &root, &errors) && root.isMember(JSON_FIELD_WIRELESS))
    {
        msg_.connected = true;
        return eAntennaCode::SUCCESS;
    }
    else
    {
        return eAntennaCode::FAILURE_PARSING_ERROR;
    }
}