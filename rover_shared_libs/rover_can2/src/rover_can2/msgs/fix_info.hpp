#ifndef FIX_INFO_HPP
#define FIX_INFO_HPP

#include "rover_can2/msgs/msg.hpp"
#include "rover_can2/helpers.hpp"

DEFINE_LOG_NODE(FixInfo_msg, Logger::eNodeState::OFF)

namespace RoverCan2::Msgs
{
    class FixInfo : public Msg<FixInfo>
    {
      public:
        enum class eMsgContentID : uint8_t
        {
            FIX_QUALITY,
            SATELLITE_COUNT,
            eLAST,
        };

      private:
        struct sMsgData
        {
            uint8_t fixQuality;
            uint8_t satelliteCount;
        };

        static constexpr CompileTimeArray<eMsgContentID, TO_UNDERLYING(eMsgContentID::eLAST)> VALID_MSG_IDS
            = {eMsgContentID::FIX_QUALITY, eMsgContentID::SATELLITE_COUNT};

      public:
        FixInfo():
            Msg(Constant::eMsgId::FIX_INFO)
        {
            _data.fixQuality = static_cast<decltype(_data.fixQuality)>(0);
            _data.satelliteCount = static_cast<decltype(_data.satelliteCount)>(0);
        }

        eLoadMsgCode _loadMsg(const CanMsg& msg_)
        {
            if (msg_.getMsgID() == Constant::eMsgId::INVALID)
            {
                return eLoadMsgCode::ERROR_INVALID_MSG;
            }

            if (msg_.getMsgID() != this->getMsgId())
            {
                return eLoadMsgCode::NOT_CONCERNED;
            }

            eMsgContentID msgContentId = static_cast<eMsgContentID>(msg_.getMsgContentID());
            if (!VALID_MSG_IDS.contains(msgContentId))
            {
                LOG_DEBUG(Logger::Nodes::FixInfo_msg,
                          "Mismatch between received message and local message definition. Received msgContentId: (%u), "
                          "expected lower than (%u) and none zero",
                          TO_UNDERLYING(msgContentId),
                          TO_UNDERLYING(eMsgContentID::eLAST));
                return eLoadMsgCode::ERROR_MISMATCH;
            }

            bool success = false;
            switch (msgContentId)
            {
                case eMsgContentID::FIX_QUALITY:
                    success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.fixQuality);
                    LOG_DEBUG(Logger::Nodes::FixInfo_msg,
                              "switch (msgContentId) case eMsgContentID::FIX_QUALITY: %s",
                              success ? "success" : "failed");
                    break;

                case eMsgContentID::SATELLITE_COUNT:
                    success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.satelliteCount);
                    LOG_DEBUG(Logger::Nodes::FixInfo_msg,
                              "switch (msgContentId) case eMsgContentID::SATELLITE_COUNT: %s",
                              success ? "success" : "failed");
                    break;

                default:
                    return eLoadMsgCode::ERROR_IMPLEMENTATION;
            }

            if (!success)
            {
                return eLoadMsgCode::ERROR_MISMATCH;
            }

            if (Helpers::MSG_CONTENT_IS_LAST_ELEM<eMsgContentID>(msg_))
            {
                return eLoadMsgCode::SUCCESS_COMPLETE;
            }
            else
            {
                return eLoadMsgCode::SUCCESS_INCOMPLETE;
            }
        }

        std::optional<CanMsg> _getCanMsg(const uint8_t msgContentId_) const
        {
            eMsgContentID msgContentID = static_cast<eMsgContentID>(msgContentId_);

            if (!VALID_MSG_IDS.contains(msgContentID))
            {
                return std::nullopt;
            }

            CanMsg msg_;
            switch (static_cast<eMsgContentID>(msgContentId_))
            {
                case eMsgContentID::FIX_QUALITY:
                    Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.fixQuality, msg_);
                    break;

                case eMsgContentID::SATELLITE_COUNT:
                    Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.satelliteCount, msg_);
                    break;

                case eMsgContentID::eLAST:
                    return std::nullopt;
            }

            return msg_;
        }

        uint8_t _getMsgContentCount(void) const
        {
            return TO_UNDERLYING(eMsgContentID::eLAST);
        }

        sMsgData& data(void)
        {
            return _data;
        }

        const sMsgData& getData(void) const
        {
            return static_cast<const sMsgData&>(_data);
        }

      private:
        sMsgData _data;
    };

}  // namespace RoverCan2::Msgs

#endif  // FIX_INFO_HPP
