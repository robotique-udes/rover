#ifndef GET_STATUS_HPP
#define GET_STATUS_HPP

#include "../antenna_command.hpp"
#include <string>

namespace Command::Base
{
    /**
     * @brief This class is responsible for getting the rssi data from the antenna
     *
     */
    class GetStatus : public AntennaCommand
    {
      private:
        static constexpr const char* STATUS_PAGE = "/status.cgi";
        static constexpr char const* JSON_FIELD_WIRELESS = "wireless";
        static constexpr char const* JSON_FIELD_RSSI = "rssi";

      public:
        GetStatus(const std::string& apiUrl_);
        ~GetStatus() override = default;
        sCommandResult execute(std::shared_ptr<cpr::Session> session_, sSignalInfos& msg_) override;

      private:
        sCommandResult getHTTPS(std::shared_ptr<cpr::Session> session_, cpr::Response& response_);
        sCommandResult parseResponse(const cpr::Response& response_, sSignalInfos& msg_);
    };

}  // namespace Command::Base

#endif  // GET_STATUS_HPP