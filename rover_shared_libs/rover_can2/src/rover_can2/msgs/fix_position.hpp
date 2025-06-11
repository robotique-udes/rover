#ifndef ROVER_CAN2_MSGS_FIX_POSITION_HPP
#define ROVER_CAN2_MSGS_FIX_POSITION_HPP

#include "rover_can2/msgs/msg.hpp"
#include "rover_can2/helpers.hpp"

DEFINE_LOG_NODE(FixPosition_msg, Logger::eNodeState::OFF)

namespace RoverCan2::Msgs
{
    class FixPosition : public Msg<FixPosition>
    {
      public:
        enum class eMsgContentID : uint8_t
        {
            LATITUDE,
            LONGITUDE,
            eLAST,
        };

      private:
        struct sMsgData
        {
            float latitude;
            float longitude;
        };

        static constexpr CompileTimeArray<eMsgContentID, TO_UNDERLYING(eMsgContentID::eLAST)> VALID_MSG_IDS
            = {eMsgContentID::LATITUDE, eMsgContentID::LONGITUDE};

      public:
        FixPosition():
            Msg(Constant::eMsgId::FIX_POSITION)
        {
            _data.latitude = static_cast<decltype(_data.latitude)>(0);
            _data.longitude = static_cast<decltype(_data.longitude)>(0);
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
                LOG_DEBUG(Logger::Nodes::FixPosition_msg,
                          "Mismatch between received message and local message definition. Received msgContentId: (%u), "
                          "expected lower than (%u) and none zero",
                          TO_UNDERLYING(msgContentId),
                          TO_UNDERLYING(eMsgContentID::eLAST));
                return eLoadMsgCode::ERROR_MISMATCH;
            }

            bool success = false;
            switch (msgContentId)
            {
                case eMsgContentID::LATITUDE:
                    success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.latitude);
                    LOG_DEBUG(Logger::Nodes::FixPosition_msg,
                              "switch (msgContentId) case eMsgContentID::LATITUDE: %s",
                              success ? "success" : "failed");
                    break;

                case eMsgContentID::LONGITUDE:
                    success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.longitude);
                    LOG_DEBUG(Logger::Nodes::FixPosition_msg,
                              "switch (msgContentId) case eMsgContentID::LONGITUDE: %s",
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
                case eMsgContentID::LATITUDE:
                    Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.latitude, msg_);
                    break;

                case eMsgContentID::LONGITUDE:
                    Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.longitude, msg_);
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

#endif  // ROVER_CAN2_MSGS_FIX_POSITION_HPP
