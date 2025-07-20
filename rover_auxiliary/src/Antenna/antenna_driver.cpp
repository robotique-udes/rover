#include "antenna_driver.hpp"
#include "rover_lib2/helpers/assert.hpp"
#include <iostream>
#include <charconv>
#include <json/json.h>

AntennaDriver::AntennaDriver(uint64_t publisherPeriodMs_):
    _session(std::make_shared<cpr::Session>()),
    _publisherPeriodMs(publisherPeriodMs_),
    _login(BASE_URL)
{
    this->setupSession();

    // Always put validate auth first
    _commands.emplace_back(std::make_unique<Command::Base::ValidateAuth>(BASE_URL));
    _commands.emplace_back(std::make_unique<Command::Base::GetStatus>(BASE_URL));
    _commands.emplace_back(std::make_unique<Command::Base::GetInterfaceStats>(BASE_URL, _publisherPeriodMs));

    ASSERT_COND_MSG(dynamic_cast<Command::Base::ValidateAuth*>(_commands.front().get()), "First command must be Validate Auth");
}

sCommandResult AntennaDriver::setUserInfo(const std::string& username_, const std::string& password_)
{
    sCommandResult result;
    sAntennaMsg msg;
    _login.setUserInfo(username_, password_);
    result = _login.execute(_session, msg);
    return result;
}

sCommandResult AntennaDriver::CbAntennaPublisher(sAntennaMsg& msg_)
{
    sCommandResult result;

    for (std::unique_ptr<AntennaCommand>& cmd : _commands)
    {
        result = cmd->execute(_session, msg_);
        if (!result)
        {
            if (result.error == "Session expired")
            {
                result = this->handleDisconnect(msg_);
            }
            else
            {
                msg_.clear();
            }

            return result;
        }
    }

    return result;
}

sCommandResult AntennaDriver::handleDisconnect(sAntennaMsg& msg_)
{
    sCommandResult result;
    uint8_t loginAttemps;
    for (loginAttemps = 0; loginAttemps < MAX_LOGIN_ATTEMPTS && !result; loginAttemps++)
    {
        _login.execute(_session, msg_);
    }

    if (!result)
    {
        msg_.clear();
        msg_.connected = false;
        return result;
    }

    for (std::unique_ptr<AntennaCommand>& cmd : _commands)
    {
        result = cmd->execute(_session, msg_);
        if (!result)
        {
            return result;
        }
    }
    return result;
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
