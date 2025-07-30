#ifndef ROVER_CAN2_MSGS_PWM_INFO_HPP
#define ROVER_CAN2_MSGS_PWM_INFO_HPP

#include "rover_can2/msgs/msg.hpp"
#include "rover_can2/helpers.hpp"

DEFINE_LOG_NODE(PwmInfo_msg, Logger::eNodeState::OFF)

namespace RoverCan2::Msgs
{
    class PwmInfo : public Msg<PwmInfo>
    {
      public:
        enum class eMsgContentID : uint8_t
        {
            FREQUENCY_CTRL_EN,
            DUTY_CYCLE_CTRL_EN,
            eLAST,
        };

      private:
        struct sMsgData
        {
            bool frequencyCtrlEn;
            bool dutyCycleCtrlEn;
        };

        static constexpr CompileTimeArray<eMsgContentID, TO_UNDERLYING(eMsgContentID::eLAST)> VALID_MSG_IDS
            = {eMsgContentID::FREQUENCY_CTRL_EN, eMsgContentID::DUTY_CYCLE_CTRL_EN};

      public:
        PwmInfo():
            Msg(Constant::eMsgId::PWM_INFO)
        {
            _data.frequencyCtrlEn = static_cast<decltype(_data.frequencyCtrlEn)>(0);
            _data.dutyCycleCtrlEn = static_cast<decltype(_data.dutyCycleCtrlEn)>(0);
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
                LOG_DEBUG(Logger::Nodes::PwmInfo_msg,
                          "Mismatch between received message and local message definition. Received msgContentId: (%u), "
                          "expected lower than (%u) and none zero",
                          TO_UNDERLYING(msgContentId),
                          TO_UNDERLYING(eMsgContentID::eLAST));
                return eLoadMsgCode::ERROR_MISMATCH;
            }

            bool success = false;
            switch (msgContentId)
            {
                case eMsgContentID::FREQUENCY_CTRL_EN:
                    success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.frequencyCtrlEn);
                    LOG_DEBUG(Logger::Nodes::PwmInfo_msg,
                              "switch (msgContentId) case eMsgContentID::FREQUENCY_CTRL_EN: %s",
                              success ? "success" : "failed");
                    break;

                case eMsgContentID::DUTY_CYCLE_CTRL_EN:
                    success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.dutyCycleCtrlEn);
                    LOG_DEBUG(Logger::Nodes::PwmInfo_msg,
                              "switch (msgContentId) case eMsgContentID::DUTY_CYCLE_CTRL_EN: %s",
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
                case eMsgContentID::FREQUENCY_CTRL_EN:
                    Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.frequencyCtrlEn, msg_);
                    break;

                case eMsgContentID::DUTY_CYCLE_CTRL_EN:
                    Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.dutyCycleCtrlEn, msg_);
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

#endif  // ROVER_CAN2_MSGS_PWM_INFO_HPP
