#ifndef ANTENNA_DRIVER_HPP
#define ANTENNA_DRIVER_HPP

#include <string>
#include <functional>
#include <memory>
#include <rover_lib2/helpers/constants.hpp>
#include <rover_lib2/helpers/time.hpp>
#include <rover_lib2/helpers/one_shot_timer.hpp>
#include <cpr/cpr.h>
#include "antenna_commands/antenna_command.hpp"
#include "antenna_commands/base/getStatus.hpp"
#include "antenna_commands/base/getInterfaceStats.hpp"
#include "antenna_commands/base/login.hpp"
#include "antenna_commands/base/validateAuth.hpp"

class AntennaDriver
{
    static constexpr const char* BASE_URL = Constants::AntennaInfo::getURL(Constants::AntennaInfo::eAntennaType::Base);
    static_assert(BASE_URL != nullptr, "Base url can't be nullptr");

    static constexpr const char* HTTP_CIPHER = "DEFAULT@SECLEVEL=1";
    static constexpr uint8_t MAX_LOGIN_ATTEMPTS = 3U;
    static constexpr uint16_t SESSION_CONNECT_TIMEOUT_MS = 500;
    static constexpr uint16_t SESSION_TIMEOUT_MS = 1000;
    static constexpr uint64_t LOGIN_COOLDOWN_MS = 60000U;

  public:
    AntennaDriver(uint64_t publisherPeriodMs_);
    sCommandResult setUserInfo(const std::string& username_, const std::string& password_);
    sCommandResult CbAntennaPublisher(sAntennaMsg& msg_);

  private:
    /**
     * @brief Specific session for the M2 rocket Antenna
     *
     */
    void setupSession(void);
    sCommandResult handleDisconnect(sAntennaMsg& msg_);

    std::shared_ptr<cpr::Session> _session;
    uint64_t _publisherPeriodMs;

    std::string _username;
    std::string _password;

    Command::Base::Login _login;
    std::vector<std::unique_ptr<AntennaCommand>> _commands;

    OneShotTimer<uint64_t, &Time::millis> _loginCooldownTimer;
    bool _cooldownActive = false;
};

#endif  // ANTENNA_DRIVER_HPP