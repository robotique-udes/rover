#ifndef ROVER_CAN2_MSGS_PTZ_CMD_HPP
#define ROVER_CAN2_MSGS_PTZ_CMD_HPP

#include "rover_can2/msgs/msg.hpp"
#include "rover_can2/helpers.hpp"

DEFINE_LOG_NODE(PtzCmd_msg, Logger::eNodeState::OFF)

namespace RoverCan2::Msgs
{
    class PtzCmd : public Msg<PtzCmd>
    {
      public:
        enum class eMsgContentID : uint8_t
        {
            PAN,
            TILT,
            ZOOM,
            eLAST,
        };

      private:
        struct sMsgData
        {
            float pan;
            float tilt;
            float zoom;
        };

        static constexpr CompileTimeArray<eMsgContentID, TO_UNDERLYING(eMsgContentID::eLAST)> VALID_MSG_IDS
            = {eMsgContentID::PAN, eMsgContentID::TILT, eMsgContentID::ZOOM};

      public:
        PtzCmd():
            Msg(Constant::eMsgId::PTZ_CMD)
        {
            _data.pan = static_cast<decltype(_data.pan)>(0);
            _data.tilt = static_cast<decltype(_data.tilt)>(0);
            _data.zoom = static_cast<decltype(_data.zoom)>(0);
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
                LOG_DEBUG(Logger::Nodes::PtzCmd_msg,
                          "Mismatch between received message and local message definition. Received msgContentId: (%u), "
                          "expected lower than (%u) and none zero",
                          TO_UNDERLYING(msgContentId),
                          TO_UNDERLYING(eMsgContentID::eLAST));
                return eLoadMsgCode::ERROR_MISMATCH;
            }

            bool success = false;
            switch (msgContentId)
            {
                case eMsgContentID::PAN:
                    success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.pan);
                    LOG_DEBUG(Logger::Nodes::PtzCmd_msg,
                              "switch (msgContentId) case eMsgContentID::PAN: %s",
                              success ? "success" : "failed");
                    break;

                case eMsgContentID::TILT:
                    success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.tilt);
                    LOG_DEBUG(Logger::Nodes::PtzCmd_msg,
                              "switch (msgContentId) case eMsgContentID::TILT: %s",
                              success ? "success" : "failed");
                    break;

                case eMsgContentID::ZOOM:
                    success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.zoom);
                    LOG_DEBUG(Logger::Nodes::PtzCmd_msg,
                              "switch (msgContentId) case eMsgContentID::ZOOM: %s",
                              success ? "success" : "failed");
                    break;

                case eMsgContentID::eLAST:
                    [[fallthrough]];
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
                case eMsgContentID::PAN:
                    Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.pan, msg_);
                    break;

                case eMsgContentID::TILT:
                    Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.tilt, msg_);
                    break;

                case eMsgContentID::ZOOM:
                    Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.zoom, msg_);
                    break;

                case eMsgContentID::eLAST:
                    [[fallthrough]];

                default:
                    return std::nullopt;
                    break;
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

#endif  // ROVER_CAN2_MSGS_PTZ_CMD_HPP
