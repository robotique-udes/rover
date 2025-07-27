#ifndef GET_STATUS_HPP
#define GET_STATUS_HPP

#include "../antenna_command.hpp"
#include "../antenna_msg.hpp"
#include <string>
#include <cpr/cpr.h>

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
        GetStatus(const std::string& apiUrl_, std::shared_ptr<cpr::Session> session_);
        ~GetStatus() override = default;
        eAntennaCode execute(void) override;
        float getRssi(void) const;

      private:
        eAntennaCode getHTTPS(std::shared_ptr<cpr::Session> session_, cpr::Response& response_) const;
        eAntennaCode parseResponse(const cpr::Response& response_);

        std::shared_ptr<cpr::Session> _session;
        float _rssi = 0.0f;
    };

}  // namespace Command::Base

#endif  // GET_STATUS_HPP