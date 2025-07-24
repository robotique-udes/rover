#include "get_interface_stats.hpp"
#include <json/json.h>
#include <charconv>

Command::Base::GetInterfaceStats::GetInterfaceStats(const std::string& apiUrl_, uint64_t publisherPeriodMs_):
    AntennaCommand(apiUrl_),
    _publisherPeriodMs(publisherPeriodMs_)
{
}

eAntennaCode Command::Base::GetInterfaceStats::execute(std::shared_ptr<cpr::Session> session_, sSignalInfos& msg_)
{
    cpr::Response response;
    eAntennaCode result = this->getHTTPS(session_, response);
    if (result != eAntennaCode::SUCCESS)
    {
        return result;
    }

    result = this->parseResponse(response, msg_);
    return result;
}

eAntennaCode Command::Base::GetInterfaceStats::getHTTPS(std::shared_ptr<cpr::Session> session_, cpr::Response& response_)
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

eAntennaCode Command::Base::GetInterfaceStats::parseResponse(const cpr::Response& response_, sSignalInfos& msg_)
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
                        if (!updateRate(stats[JSON_FIELD_RX_BYTES].asString(), _wlanRxBytes, msg_.rxRate))
                        {
                            return eAntennaCode::FAILURE_PARSING_ERROR;
                        }
                    }

                    if (stats.isMember(JSON_FIELD_TX_BYTES))
                    {
                        if (!updateRate(stats[JSON_FIELD_TX_BYTES].asString(), _wlanTxBytes, msg_.txRate))
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
        rate_ = (currentBytes - lastByte_) * 1000.0f / _publisherPeriodMs;
        lastByte_ = currentBytes;
        return true;
    }
    return false;
}