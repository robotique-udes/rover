#ifndef VALIDATE_AUTH_HPP
#define VALIDATE_AUTH_HPP

#include "../antenna_command.hpp"
#include <cpr/cpr.h>

namespace Command::Base
{
    /**
     * @brief This class validates the authentification by checking if response of get status.cgi is a valid json object
     *
     */
    class ValidateAuth : public AntennaCommand
    {
      private:
        static constexpr const char* STATUS_PAGE = "/status.cgi";
        static constexpr char const* JSON_FIELD_WIRELESS = "wireless";

      public:
        ValidateAuth(const std::string& apiUrl_, std::shared_ptr<cpr::Session> session_);
        ~ValidateAuth() override = default;
        eAntennaCode execute(void) override;
        bool getConnectedStatus(void);

      private:
        std::shared_ptr<cpr::Session> _session;
        eAntennaCode getHTTPS(std::shared_ptr<cpr::Session> session_, cpr::Response& response_);
        eAntennaCode validateFormat(const cpr::Response& response_);
        bool _connected = false;
    };
}  // namespace Command::Base

#endif  // VALIDATE_AUTH_CPP