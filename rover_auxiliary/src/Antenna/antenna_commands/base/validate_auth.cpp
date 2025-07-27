#include "validate_auth.hpp"
#include <json/json.h>

Command::Base::ValidateAuth::ValidateAuth(const std::string& apiUrl_, std::shared_ptr<cpr::Session> session_):
    AntennaCommand(apiUrl_),
    _session(session_)
{
}

eAntennaCode Command::Base::ValidateAuth::execute(void)
{
    cpr::Response response;
    eAntennaCode result = this->getHTTPS(_session, response);
    if (result != eAntennaCode::SUCCESS)
    {
        _connected = false;
        return result;
    }

    result = this->validateFormat(response);

    if (result != eAntennaCode::SUCCESS)
    {
        _connected = false;
    }
    return result;
}

eAntennaCode Command::Base::ValidateAuth::getHTTPS(std::shared_ptr<cpr::Session> session_, cpr::Response& response_) const
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

eAntennaCode Command::Base::ValidateAuth::validateFormat(const cpr::Response& response_)
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
        _connected = true;
        return eAntennaCode::SUCCESS;
    }
    else
    {
        return eAntennaCode::FAILURE_PARSING_ERROR;
    }
}

bool Command::Base::ValidateAuth::getConnectedStatus(void) const
{
    return _connected;
}