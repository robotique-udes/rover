#ifndef ARM_POSITION_STATUS_HPP
#define ARM_POSITION_STATUS_HPP

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
            POSITION,
            eLAST
        };

      private:
        struct sMsgData
        {
            bool jointLimitReached;
        };

        static constexpr CompileTimeArray<eMsgContentID, TO_UNDERLYING(eMsgContentID::eLAST)> VALID_MSG_IDS
            = {eMsgContentID::POSITION};

        sMsgData _data;

      public:
        ArmPositionStatus():
            Msg(Constant::eMsgId::ARM_POSITION_STATUS)
        {
            _data.jointLimitReached = false;
        }

        eLoadMsgCode _loadMsg(const CanMsg& msg_) override
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
                LOG_DEBUG(Logger::Nodes::ArmInfo_msg,
                          "Unexpected msgContentId: %u (expected < %u)",
                          TO_UNDERLYING(contentId),
                          TO_UNDERLYING(eMsgContentID::eLAST));
                return eLoadMsgCode::ERROR_MISMATCH;
            }

            bool success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.jointLimitReached);
            LOG_DEBUG(Logger::Nodes::ArmInfo_msg, "jointLimitReached parsing: %s", success ? "success" : "failure");
            if (!success)
            {
                return eLoadMsgCode::ERROR_MISMATCH;
            }

            return Helpers::MSG_CONTENT_IS_LAST_ELEM<eMsgContentID>(msg_) ? eLoadMsgCode::SUCCESS_COMPLETE
                                                                          : eLoadMsgCode::SUCCESS_INCOMPLETE;
        }

        std::optional<CanMsg> _getCanMsg(uint8_t msgContentId_) const override
        {
            eMsgContentID contentId = static_cast<eMsgContentID>(msgContentId_);
            if (!VALID_MSG_IDS.contains(contentId))
            {
                return std::nullopt;
            }
            CanMsg msg;
            Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.jointLimitReached, msg);
            return msg;
        }

        uint8_t _getMsgContentCount() const override
        {
            return TO_UNDERLYING(eMsgContentID::eLAST);
        }

        bool isJointLimitReached() const
        {
            return _data.jointLimitReached;
        }
        void setJointLimitReached(bool reached)
        {
            _data.jointLimitReached = reached;
        }
    };

}  // namespace RoverCan2::Msgs

#endif  // ARM_INFO_HPP
