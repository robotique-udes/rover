#ifndef GET_STATUS_HPP
#define GET_STATUS_HPP

#include "../antenna_command.hpp"
#include <string>

namespace Command
{
    class GetStatus : public AntennaCommand
    {
      private:
        static constexpr const char* STATUS_PAGE = "/status.cgi";
        static constexpr char const* JSON_FIELD_WIRELESS = "wireless";
        static constexpr char const* JSON_FIELD_RSSI = "rssi";

      public:
        GetStatus(const std::string baseURL_);
        ~GetStatus() override = default;
        sCommandResult execute(std::shared_ptr<cpr::Session> session_, sAntennaMsg& msg_);

      private:
        sCommandResult getHTTPS(std::shared_ptr<cpr::Session> session_, cpr::Response& response);
        sCommandResult parseResponse(const cpr::Response& response, sAntennaMsg& msg_);

        std::string _baseURL;
    };

}  // namespace Command

#endif  // GET_STATUS_HPP