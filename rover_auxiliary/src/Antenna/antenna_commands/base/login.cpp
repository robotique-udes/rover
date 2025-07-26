#include "login.hpp"
Command::Base::Login::Login(const std::string& apiUrl_, const std::string& username_, const std::string& password_, std::shared_ptr<cpr::Session> session_):
    AntennaCommand(apiUrl_),
    _session(session_),
    _username(username_),
    _password(password_)
{
}

eAntennaCode Command::Base::Login::execute(void)
{
    return this->postHTTPS(_session);
}

eAntennaCode Command::Base::Login::postHTTPS(std::shared_ptr<cpr::Session> session_)
{
    session_->SetUrl(cpr::Url{this->getApiUrl() + LOGIN_PAGE});
    cpr::Payload payload{{"username", _username}, {"password", _password}};
    session_->SetOption(payload);
    cpr::Response response = session_->Post();

    session_->SetOption(cpr::Payload{});  // important to always clear payload after a post
    switch (response.status_code)
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
            return eAntennaCode::FAILURE_SESSION_EXPIRED;
            break;
        default:
            return eAntennaCode::FAILURE_UNKNOWN;
            break;
    }
}