#include "antenna_driver.hpp"
#include "rover_lib2/helpers/assert.hpp"

AntennaDriver::AntennaDriver(uint64_t publisherPeriodMs_, const std::string& username_, const std::string& password_):
    _session(std::make_shared<cpr::Session>()),
    _publisherPeriodMs(publisherPeriodMs_),
    _login(BASE_URL, username_, password_, _session),
    _loginCooldownTimer(LOGIN_COOLDOWN_MS)
{
    this->setupSession();

    _validateAuth = std::make_shared<Command::Base::ValidateAuth>(BASE_URL, _session);
    _getStatus = std::make_shared<Command::Base::GetStatus>(BASE_URL, _session);
    _getInterfaceStats = std::make_shared<Command::Base::GetInterfaceStats>(BASE_URL, _publisherPeriodMs, _session);
    _commands[0] = _validateAuth;
    _commands[1] = _getStatus;
    _commands[2] = _getInterfaceStats;
}

eAntennaCode AntennaDriver::retrieveDatalinkInfos(sSignalInfos& msg_)
{
    eAntennaCode result;

    for (const std::weak_ptr<AntennaCommand>& cmd : _commands)
    {
        if (std::shared_ptr<AntennaCommand> lockedCmd = cmd.lock())
        {
            result = lockedCmd->execute();
        }
        else
        {
            result = eAntennaCode::FAILURE_UNKNOWN;
        }
        switch (result)
        {
            case eAntennaCode::SUCCESS:
                break;

            case eAntennaCode::FAILURE_SESSION_EXPIRED:
                [[fallthrough]];
            case eAntennaCode::FAILURE_PARSING_ERROR:
                [[fallthrough]];
            case eAntennaCode::FAILURE_DEVICE_OFFLINE:
                result = this->handleDisconnect(msg_);
                return result;
                break;
            default:
                msg_ = sSignalInfos{};
                msg_.connected = false;
                return result;
                break;
        }
    }

    msg_.connected = _validateAuth->getConnectedStatus();
    msg_.rssi = _getStatus->getRssi();
    msg_.rxRate = _getInterfaceStats->getRxRate();
    msg_.txRate = _getInterfaceStats->getTxRate();
    return result;
}

eAntennaCode AntennaDriver::handleDisconnect(sSignalInfos& msg_)
{
    eAntennaCode result = eAntennaCode::FAILURE_UNKNOWN;
    if (_cooldownActive && !_loginCooldownTimer.isReady())
    {
        msg_ = sSignalInfos{};
        msg_.connected = false;
        return eAntennaCode::FAILURE_ON_COOLDOWN;
    }
    _cooldownActive = false;

    for (uint8_t loginAttempts = 0; loginAttempts < MAX_LOGIN_ATTEMPTS && result != eAntennaCode::SUCCESS; loginAttempts++)
    {
        result = _login.execute();
    }

    if (result != eAntennaCode::SUCCESS)
    {
        _loginCooldownTimer = OneShotTimer<uint64_t, &Time::millis>{LOGIN_COOLDOWN_MS};
        _cooldownActive = true;
        msg_ = sSignalInfos{};
        msg_.connected = false;
        return result;
    }

    for (const std::weak_ptr<AntennaCommand>& cmd : _commands)
    {
        if (std::shared_ptr<AntennaCommand> lockedCmd = cmd.lock())
        {
            result = lockedCmd->execute();
        }
        else
        {
            result = eAntennaCode::FAILURE_UNKNOWN;
        }

        if (result != eAntennaCode::SUCCESS)
        {
            _loginCooldownTimer = OneShotTimer<uint64_t, &Time::millis>{LOGIN_COOLDOWN_MS};
            _cooldownActive = true;
            return result;
        }
    }

    msg_.connected = _validateAuth->getConnectedStatus();
    msg_.rssi = _getStatus->getRssi();
    msg_.rxRate = _getInterfaceStats->getRxRate();
    msg_.txRate = _getInterfaceStats->getTxRate();
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
