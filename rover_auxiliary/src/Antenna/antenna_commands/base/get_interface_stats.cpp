#include "get_interface_stats.hpp"
#include <json/json.h>
#include <charconv>

Command::Base::GetInterfaceStats::GetInterfaceStats(const std::string& apiUrl_,
                                                    uint64_t publisherPeriodMs_,
                                                    std::shared_ptr<cpr::Session> session_):
    AntennaCommand(apiUrl_),
    _session(session_),
    _publisherPeriodMs(publisherPeriodMs_)
{
}

eAntennaCode Command::Base::GetInterfaceStats::execute(void)
{
    cpr::Response response;
    eAntennaCode result = this->getHTTPS(_session, response);
    if (result != eAntennaCode::SUCCESS)
    {
        return result;
    }

    result = this->parseResponse(response);
    return result;
}

eAntennaCode Command::Base::GetInterfaceStats::getHTTPS(std::shared_ptr<cpr::Session> session_, cpr::Response& response_) const
{
    session_->SetUrl(cpr::Url{this->getApiUrl() + IFSTATS_PAGE});
    response_ = session_->Get();

    switch (response_.status_code)
    {
        case std::to_underlying(eHttpStatus::OK):
            return eAntennaCode::SUCCESS;
            break;

        case std::to_underlying(eHttpStatus::FORBIDDEN):
            [[fallthrough]];
        case std::to_underlying(eHttpStatus::UNAUTHORIZED):
            return eAntennaCode::FAILURE_SESSION_EXPIRED;
            break;
        case std::to_underlying(eHttpStatus::OFFLINE):
            return eAntennaCode::FAILURE_DEVICE_OFFLINE;
            break;
        default:
            return eAntennaCode::FAILURE_UNKNOWN;
            break;
    }
}

eAntennaCode Command::Base::GetInterfaceStats::parseResponse(const cpr::Response& response_)
{
    if (response_.text.empty())
    {
        return eAntennaCode::FAILURE_PARSING_ERROR;
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
                        if (!updateRate(stats[JSON_FIELD_RX_BYTES].asString(), _wlanRxBytes, _rxRate))
                        {
                            return eAntennaCode::FAILURE_PARSING_ERROR;
                        }
                    }

                    if (stats.isMember(JSON_FIELD_TX_BYTES))
                    {
                        if (!updateRate(stats[JSON_FIELD_TX_BYTES].asString(), _wlanTxBytes, _txRate))
                        {
                            return eAntennaCode::FAILURE_PARSING_ERROR;
                        }
                    }
                }
            }
        }
        return eAntennaCode::SUCCESS;
    }
    else
    {
        return eAntennaCode::FAILURE_PARSING_ERROR;
    }
}

bool Command::Base::GetInterfaceStats::updateRate(const std::string& byteStr_, uint64_t& lastByte_, float& rate_)
{
    uint64_t currentBytes;
    std::from_chars_result charResult = std::from_chars(byteStr_.data(), byteStr_.data() + byteStr_.size(), currentBytes);

    if (charResult.ec == std::errc{})
    {
        rate_ = static_cast<float>(currentBytes - lastByte_) * 1000.0f / static_cast<float>(_publisherPeriodMs);
        lastByte_ = currentBytes;
        return true;
    }
    return false;
}

float Command::Base::GetInterfaceStats::getRxRate(void) const
{
    return _rxRate;
}

float Command::Base::GetInterfaceStats::getTxRate(void) const
{
    return _txRate;
}
