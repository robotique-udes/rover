#ifndef GET_INTERFACE_STATS_HPP
#define GET_INTERFACE_STATS_HPP

#include "../antenna_command.hpp"
#include <string>
#include <cpr/cpr.h>

namespace Command::Base
{
    /**
     * @brief This class is responsible for getting the wlan rates from the antenna interface statistics
     *
     */
    class GetInterfaceStats : public AntennaCommand
    {
      private:
        static constexpr const char* IFSTATS_PAGE = "/ifstats.cgi";
        static constexpr char const* JSON_FIELD_INTERFACES = "interfaces";
        static constexpr char const* JSON_FIELD_STATS = "stats";
        static constexpr char const* JSON_FIELD_RX_BYTES = "rx_bytes";
        static constexpr char const* JSON_FIELD_TX_BYTES = "tx_bytes";
        static constexpr uint8_t INTERFACE_WLAN_INDEX = 0;
        static constexpr uint8_t INTERFACE_LAN_INDEX = 1;

      public:
        GetInterfaceStats(const std::string& apiUrl_, uint64_t publisherPeriodMs_, std::shared_ptr<cpr::Session> session_);
        ~GetInterfaceStats() override = default;
        eAntennaCode execute(void) override;
        float getRxRate(void);
        float getTxRate(void);

      private:
        eAntennaCode getHTTPS(std::shared_ptr<cpr::Session> session_, cpr::Response& response_);
        eAntennaCode parseResponse(const cpr::Response& response_);
        bool updateRate(const std::string& byteStr_, uint64_t& lastByte_, float& rate_);

        std::shared_ptr<cpr::Session> _session;

        uint8_t _loginAttempts = 0;
        uint64_t _lanRxBytes = 0;
        uint64_t _lanTxBytes = 0;
        uint64_t _wlanRxBytes = 0;
        uint64_t _wlanTxBytes = 0;
        uint64_t _publisherPeriodMs;

        float _rxRate = 0.0f;
        float _txRate = 0.0f;
    };

}  // namespace Command::Base

#endif  // GET_INTERFACE_STATS_HPP