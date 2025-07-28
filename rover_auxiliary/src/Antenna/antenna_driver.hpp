#ifndef ANTENNA_DRIVER_HPP
#define ANTENNA_DRIVER_HPP

#include <memory>
#include <rover_lib2/helpers/constants.hpp>
#include <rover_lib2/helpers/time.hpp>
#include <rover_lib2/helpers/one_shot_timer.hpp>
#include <cpr/cpr.h>
#include "antenna_commands/antenna_command.hpp"
#include "antenna_commands/base/get_status.hpp"
#include "antenna_commands/base/get_interface_stats.hpp"
#include "antenna_commands/base/login.hpp"
#include "antenna_commands/base/validate_auth.hpp"

class AntennaDriver
{
  private:
    static constexpr const char* BASE_URL = Constants::AntennaInfo::getURL<Constants::AntennaInfo::eAntennaType::BASE>();

    static constexpr const char* HTTP_CIPHER = "DEFAULT@SECLEVEL=1";
    static constexpr uint8_t MAX_LOGIN_ATTEMPTS = 3U;
    static constexpr uint16_t SESSION_CONNECT_TIMEOUT_MS = 500U;
    static constexpr uint16_t SESSION_TIMEOUT_MS = 1'000U;
    static constexpr uint64_t LOGIN_COOLDOWN_MS = 60'000UL;

  public:
    AntennaDriver(uint64_t publisherPeriodMs_, const std::string& username_, const std::string& password_);
    eAntennaCode retrieveDatalinkInfos(sSignalInfos& msg_);

  private:
    /**
     * @brief Specific session for the M2 rocket Antenna
     *
     */
    void setupSession(void) const;
    eAntennaCode handleDisconnect(sSignalInfos& msg_);
    void startCooldown(void);

    std::shared_ptr<cpr::Session> _session;
    uint64_t _publisherPeriodMs;

    Command::Base::Login _login;
    std::shared_ptr<Command::Base::ValidateAuth> _validateAuth;
    std::shared_ptr<Command::Base::GetStatus> _getStatus;
    std::shared_ptr<Command::Base::GetInterfaceStats> _getInterfaceStats;
    std::array<std::weak_ptr<AntennaCommand>, 3> _commands;

    OneShotTimer<uint64_t, &Time::millis> _loginCooldownTimer;
    bool _cooldownActive = false;
};

#endif  // ANTENNA_DRIVER_HPP