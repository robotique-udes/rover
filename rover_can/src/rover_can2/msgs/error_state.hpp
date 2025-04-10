#ifndef ERROR_STATE_HPP
#define ERROR_STATE_HPP

#include "rover_can2/msgs/msg.hpp"
#include "rover_can2/helpers.hpp"

DEFINE_LOG_NODE(ErrorState_msg, Logger::eNodeState::OFF)

namespace RoverCan2::Msgs
{
    class ErrorState : public Msg<ErrorState>
    {
      public:
        enum class eMsgContentID : uint8_t
        {
            ERROR,
            eLAST,
        };

      private:
        struct sMsgData
        {
            bool error;
        };

        static constexpr CompileTimeArray<eMsgContentID, TO_UNDERLYING(eMsgContentID::eLAST)> VALID_MSG_IDS
            = {eMsgContentID::ERROR};

      public:
        ErrorState();

        eLoadMsgCode _loadMsg(const CanMsg& msg_);
        std::optional<CanMsg> _getCanMsg(const uint8_t msgContentId_) const;
        uint8_t _getMsgContentCount(void) const;
        sMsgData& data(void);

      private:
        sMsgData _data;
    };

    ErrorState::ErrorState():
        Msg(Constant::eMsgId::ERROR_STATE)
    {
        _data.error = static_cast<decltype(_data.error)>(0);
    }

    eLoadMsgCode ErrorState::_loadMsg(const CanMsg& msg_)
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
            LOG_DEBUG(Logger::Nodes::ErrorState_msg,
                      "Missmatch between received message and local message definition. Received msgContentId: (%u), "
                      "expected lower than (%u) and none zero",
                      TO_UNDERLYING(msgContentId),
                      TO_UNDERLYING(eMsgContentID::eLAST));
            return eLoadMsgCode::ERROR_MISSMATCH;
        }

        bool success = false;
        switch (msgContentId)
        {
            case eMsgContentID::ERROR:
                success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.error);
                LOG_DEBUG(Logger::Nodes::ErrorState_msg,
                          "switch (msgContentId) case eMsgContentID::ERROR: %s",
                          success ? "success" : "failed");
                break;

            default:
                return eLoadMsgCode::ERROR_IMPLEMENTATION;
        }

        if (!success)
        {
            return eLoadMsgCode::ERROR_MISSMATCH;
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

    std::optional<CanMsg> ErrorState::_getCanMsg(const uint8_t msgContentId_) const
    {
        eMsgContentID msgContentID = static_cast<eMsgContentID>(msgContentId_);

        if (!VALID_MSG_IDS.contains(msgContentID))
        {
            return std::nullopt;
        }

        CanMsg msg_;
        switch (static_cast<eMsgContentID>(msgContentId_))
        {
            case eMsgContentID::ERROR:
                Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.error, msg_);
                break;

            case eMsgContentID::eLAST:
                return std::nullopt;
        }

        return msg_;
    }

    uint8_t ErrorState::_getMsgContentCount(void) const
    {
        return TO_UNDERLYING(eMsgContentID::eLAST);
    }

    ErrorState::sMsgData& ErrorState::data(void)
    {
        return _data;
    }
}  // namespace RoverCan2::Msgs

#endif  // ERROR_STATE_HPP
