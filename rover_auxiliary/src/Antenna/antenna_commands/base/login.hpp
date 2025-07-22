#ifndef LOGIN_BASE_HPP
#define LOGIN_BASE_HPP
#include <string>
#include "../antenna_command.hpp"

namespace Command::Base
{
    class Login : public AntennaCommand
    {
      private:
        static constexpr const char* LOGIN_PAGE = "/login.cgi";

      public:
        Login(const std::string& apiUrl_, const std::string& username_, const std::string& password_);
        ~Login() override = default;
        /**
         * @brief POSTs the username and password on the antenna's login page
         *
         * @note A true response only means the POST was successful not that the login was successful.
         * @note To check if credentials are valid use validateAuth  instead
         */
        sCommandResult execute(std::shared_ptr<cpr::Session> session_, sAntennaMsg& msg_) override;

      private:
        sCommandResult postHTTPS(std::shared_ptr<cpr::Session> session_);

        std::string _username;
        std::string _password;
    };

}  // namespace Command::Base

#endif  // LOGIN_BASE_HPP