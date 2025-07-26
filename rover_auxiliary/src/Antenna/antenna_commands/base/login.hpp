#ifndef LOGIN_BASE_HPP
#define LOGIN_BASE_HPP
#include <string>
#include "../antenna_command.hpp"
#include <cpr/cpr.h>

namespace Command::Base
{
    class Login : public AntennaCommand
    {
      private:
        static constexpr const char* LOGIN_PAGE = "/login.cgi";

      public:
        Login(const std::string& apiUrl_,
              const std::string& username_,
              const std::string& password_,
              std::shared_ptr<cpr::Session> session_);
        ~Login() override = default;
        /**
         * @brief POSTs the username and password on the antenna's login page
         *
         * @note A true response only means the POST was successful not that the login was successful.
         * @note To check if credentials are valid use validateAuth  instead
         */
        eAntennaCode execute(void) override;

      private:
        eAntennaCode postHTTPS(std::shared_ptr<cpr::Session> session_);

        std::shared_ptr<cpr::Session> _session;
        std::string _username;
        std::string _password;
    };

}  // namespace Command::Base

#endif  // LOGIN_BASE_HPP