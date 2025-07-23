#include "login.hpp"

Command::Base::Login::Login(const std::string& apiUrl_, const std::string& username_, const std::string& password_):
    AntennaCommand(apiUrl_),
    _username(username_),
    _password(password_)
{
}

sCommandResult Command::Base::Login::execute(std::shared_ptr<cpr::Session> session_, sSignalInfos& msg_)
{
    (void)msg_;
    return this->postHTTPS(session_);
}

sCommandResult Command::Base::Login::postHTTPS(std::shared_ptr<cpr::Session> session_)
{
    sCommandResult result;
    session_->SetUrl(cpr::Url{this->getApiUrl() + LOGIN_PAGE});
    cpr::Payload payload{{"username", _username}, {"password", _password}};
    session_->SetOption(payload);
    cpr::Response response = session_->Post();

    session_->SetOption(cpr::Payload{});  // important to always clear payload after a post

    result.httpStatus = response.status_code;
    switch (response.status_code)
    {
        case std::to_underlying(eHttpStatus::OK):
            result.success = true;
            break;

        case std::to_underlying(eHttpStatus::FORBIDDEN):
            [[fallthrough]];
        case std::to_underlying(eHttpStatus::UNAUTHORIZED):
            result.success = false;
            result.error = "Login forbidden or unauthorized: check your credentials in your .env";
            break;
        case std::to_underlying(eHttpStatus::OFFLINE):
            result.success = false;
            result.error = "Antenna is offline";
            break;
        default:
            result.success = false;
            result.error = "POST attempt was unsuccessful, unexpected http status code: " + std::to_string(response.status_code);
            break;
    }
    return result;
}