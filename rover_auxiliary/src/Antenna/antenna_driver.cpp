#include "antenna_driver.hpp"
#include "rover_lib2/helpers/assert.hpp"

namespace
{
    bool isAuthOrOfflineError(int status)
    {
        return status == std::to_underlying(eHttpStatus::FORBIDDEN) || status == std::to_underlying(eHttpStatus::UNAUTHORIZED)
               || status == std::to_underlying(eHttpStatus::OFFLINE);
    }
}  // namespace

AntennaDriver::AntennaDriver(uint64_t publisherPeriodMs_):
    _session(std::make_shared<cpr::Session>()),
    _publisherPeriodMs(publisherPeriodMs_),
    _login(BASE_URL),
    _loginCooldownTimer(LOGIN_COOLDOWN_MS)
{
    this->setupSession();

    // Always put validate auth first
    _commands[0] = (std::make_unique<Command::Base::ValidateAuth>(BASE_URL));
    _commands[1] = (std::make_unique<Command::Base::GetStatus>(BASE_URL));
    _commands[2] = (std::make_unique<Command::Base::GetInterfaceStats>(BASE_URL, _publisherPeriodMs));
}

void AntennaDriver::setUserInfo(const std::string& username_, const std::string& password_)
{
    _login.setUserInfo(username_, password_);
}

sCommandResult AntennaDriver::ExecuteAntennaCommands(sAntennaMsg& msg_)
{
    sCommandResult result;

    for (const std::unique_ptr<AntennaCommand>& cmd : _commands)
    {
        result = cmd->execute(_session, msg_);
        if (!result.success)
        {
            if (isAuthOrOfflineError(result.httpStatus))
            {
                result = this->handleDisconnect(msg_);
            }
            else
            {
                msg_ = sAntennaMsg{};
                msg_.connected = false;
            }

            return result;
        }
    }

    return result;
}

sCommandResult AntennaDriver::handleDisconnect(sAntennaMsg& msg_)
{
    sCommandResult result;
    if (_cooldownActive && !_loginCooldownTimer.isReady())
    {
        result.success = false;
        msg_ = sAntennaMsg{};
        msg_.connected = false;
        return result;
    }
    _cooldownActive = false;

    uint8_t loginAttempts;
    for (loginAttempts = 0; loginAttempts < MAX_LOGIN_ATTEMPTS && !result.success; loginAttempts++)
    {
        result = _login.execute(_session, msg_);
    }

    if (!result.success)
    {
        _loginCooldownTimer = OneShotTimer<uint64_t, &Time::millis>{LOGIN_COOLDOWN_MS};
        _cooldownActive = true;
        msg_ = sAntennaMsg{};
        msg_.connected = false;
        return result;
    }

    for (const std::unique_ptr<AntennaCommand>& cmd : _commands)
    {
        result = cmd->execute(_session, msg_);
        if (!result.success)
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

    cpr::SslOptions sslOptions;
    sslOptions.ciphers = HTTP_CIPHER;
    sslOptions.verify_peer = false;
    sslOptions.verify_host = false;
    _session->SetOption(sslOptions);

    _session->SetConnectTimeout(cpr::ConnectTimeout{SESSION_CONNECT_TIMEOUT_MS});
    _session->SetTimeout(cpr::Timeout{SESSION_TIMEOUT_MS});
}
