#include "getInterfaceStats.hpp"
#include <json/json.h>
#include <charconv>

Command::Base::GetInterfaceStats::GetInterfaceStats(const std::string& baseURL_, uint64_t publisherPeriodMs_):
    AntennaCommand(baseURL_),
    _publisherPeriodMs(publisherPeriodMs_)
{
}

sCommandResult Command::Base::GetInterfaceStats::execute(std::shared_ptr<cpr::Session> session_, sAntennaMsg& msg_)
{
    cpr::Response response;
    sCommandResult result = this->getHTTPS(session_, response);
    result.httpStatus = response.status_code;
    if (!result)
    {
        return result;
    }

    result = this->parseResponse(response, msg_);
    return result;
}

sCommandResult Command::Base::GetInterfaceStats::getHTTPS(std::shared_ptr<cpr::Session> session_, cpr::Response& response_)
{
    sCommandResult result;
    session_->SetUrl(cpr::Url{_baseURL + IFSTATS_PAGE});
    response_ = session_->Get();

    switch (response_.status_code)
    {
        case std::to_underlying(eHttpStatus::OK):
            result.success = true;
            break;

        case std::to_underlying(eHttpStatus::FORBIDDEN):
            [[fallthrough]];
        case std::to_underlying(eHttpStatus::UNAUTHORIZED):
            result.success = false;
            result.error = "Session expired";
            break;
        default:
            result.success = false;
            result.error = "Unexpected HTTP status using GET stats.cgi: " + std::to_string(response_.status_code);
            break;
    }
    return result;
}

sCommandResult Command::Base::GetInterfaceStats::parseResponse(const cpr::Response& response_, sAntennaMsg& msg_)
{
    sCommandResult result;
    if (response_.text.empty())
    {
        result.success = false;
        result.error = "Response was empty";
        return result;
    }

    Json::Value root;
    Json::CharReaderBuilder builder;
    std::string errors;

    std::istringstream stream(response_.text);
    if (Json::parseFromStream(builder, stream, &root, &errors))
    {
        if (root.isMember(JSON_FIELD_INTERFACES) && root[JSON_FIELD_INTERFACES].isArray())
        {
            Json::Value interfaces = root[JSON_FIELD_INTERFACES];

            if (interfaces.size() > 0 && interfaces[INTERFACE_WLAN_INDEX].isObject())
            {
                Json::Value interface0 = interfaces[INTERFACE_WLAN_INDEX];

                if (interface0.isMember(JSON_FIELD_STATS) && interface0[JSON_FIELD_STATS].isObject())
                {
                    Json::Value stats = interface0[JSON_FIELD_STATS];

                    if (stats.isMember(JSON_FIELD_RX_BYTES))
                    {
                        if (!updateRate(stats[JSON_FIELD_RX_BYTES].asString(), _wlanRxBytes, msg_.rxRate))
                        {
                            result.success = false;
                            result.error = "Failed to parse rx_bytes";
                            return result;
                        }
                    }

                    if (stats.isMember(JSON_FIELD_TX_BYTES))
                    {
                        if (!updateRate(stats[JSON_FIELD_TX_BYTES].asString(), _wlanTxBytes, msg_.txRate))
                        {
                            result.success = false;
                            result.error = "Failed to parse tx_bytes";
                            return result;
                        }
                    }
                }
            }
        }
        result.success = true;
    }
    else
    {
        result.success = false;
        result.error = "Unabble to parse JSON for ifStats";
    }
    return result;
}

bool Command::Base::GetInterfaceStats::updateRate(const std::string& byteStr_, uint64_t& lastByte_, float& rate_)
{
    uint64_t currentBytes;
    std::from_chars_result charResult = std::from_chars(byteStr_.data(), byteStr_.data() + byteStr_.size(), currentBytes);

    if (charResult.ec == std::errc{})
    {
        rate_ = (currentBytes - lastByte_) * 1000.0f / _publisherPeriodMs;
        lastByte_ = currentBytes;
        return true;
    }
    return false;
}