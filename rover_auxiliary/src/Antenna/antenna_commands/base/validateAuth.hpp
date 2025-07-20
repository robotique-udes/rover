#ifndef VALIDATE_AUTH_HPP
#define VALIDATE_AUTH_HPP

#include "../antenna_command.hpp"

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
        ValidateAuth(const std::string baseURL_);
        ~ValidateAuth() override = default;
        sCommandResult execute(std::shared_ptr<cpr::Session> session_, sAntennaMsg& msg_) override;

      private:
        sCommandResult getHTTPS(std::shared_ptr<cpr::Session> session_, cpr::Response& response_);
        sCommandResult validateFormat(const cpr::Response& response, sAntennaMsg& msg_);
    };
}  // namespace Command::Base

#endif  // VALIDATE_AUTH_CPP