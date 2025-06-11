#ifndef ROVER_CAN2_MSGS_ARM_POSITION_STATUS_HPP
#define ROVER_CAN2_MSGS_ARM_POSITION_STATUS_HPP

#include "rover_can2/msgs/msg.hpp"
#include "rover_can2/helpers.hpp"

DEFINE_LOG_NODE(ArmPostitionStatus_msg, Logger::eNodeState::OFF)

namespace RoverCan2::Msgs
{

    class ArmPositionStatus : public Msg<ArmPositionStatus>
    {
      public:
        enum class eMsgContentID : uint8_t
        {
            CURRENT_POSITION,
            eLAST
        };

      private:
        struct sMsgData
        {
            bool current_position;
        };

        static constexpr CompileTimeArray<eMsgContentID, TO_UNDERLYING(eMsgContentID::eLAST)> VALID_MSG_IDS
            = {eMsgContentID::CURRENT_POSITION};

        sMsgData _data;

      public:
        ArmPositionStatus():
            Msg(Constant::eMsgId::ARM_POSITION_STATUS)
        {
            _data.current_position = false;
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

            eMsgContentID contentId = static_cast<eMsgContentID>(msg_.getMsgContentID());
            if (!VALID_MSG_IDS.contains(contentId))
            {
                LOG_DEBUG(Logger::Nodes::ArmPostitionStatus_msg,
                          "Unexpected msgContentId: %u (expected < %u)",
                          TO_UNDERLYING(contentId),
                          TO_UNDERLYING(eMsgContentID::eLAST));
                return eLoadMsgCode::ERROR_MISMATCH;
            }

            bool success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.current_position);
            LOG_DEBUG(Logger::Nodes::ArmPostitionStatus_msg,
                      "switch (msgContentId) case eMsgContentID::CURRENT_POSITION: %s",
                      success ? "success" : "failed");
            if (!success)
            {
                return eLoadMsgCode::ERROR_MISMATCH;
            }

            return Helpers::MSG_CONTENT_IS_LAST_ELEM<eMsgContentID>(msg_) ? eLoadMsgCode::SUCCESS_COMPLETE
                                                                          : eLoadMsgCode::SUCCESS_INCOMPLETE;
        }

        std::optional<CanMsg> _getCanMsg(uint8_t msgContentId_) const
        {
            eMsgContentID contentId = static_cast<eMsgContentID>(msgContentId_);
            if (!VALID_MSG_IDS.contains(contentId))
            {
                return std::nullopt;
            }
            CanMsg msg;
            Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.current_position, msg);
            return msg;
        }

        uint8_t _getMsgContentCount() const
        {
            return TO_UNDERLYING(eMsgContentID::eLAST);
        }

        const sMsgData& getData(void) const
        {
            return static_cast<const sMsgData&>(_data);
        }
    };

}  // namespace RoverCan2::Msgs

#endif  // ROVER_CAN2_MSGS_ARM_POSITION_STATUS_HPP
