#ifndef ROVER_CAN2_MSGS_DDB_CMD_HPP
#define ROVER_CAN2_MSGS_DDB_CMD_HPP

#include "rover_can2/msgs/msg.hpp"
#include "rover_can2/helpers.hpp"

DEFINE_LOG_NODE(DdbCmd_msg, Logger::eNodeState::OFF)

namespace RoverCan2::Msgs
{
    class DdbCmd : public Msg<DdbCmd>
    {
      public:
        enum class eMsgContentID : uint8_t
        {
            B_0_CH_2_ON_STATE,
            B_0_CH_3_ON_STATE,
            B_1_CH_0_ON_STATE,
            B_1_CH_1_ON_STATE,
            B_1_CH_2_ON_STATE,
            B_1_CH_3_ON_STATE,
            eLAST,
        };

      private:
        struct sMsgData
        {
            bool B0_CH2_onState;
            bool B0_CH3_onState;
            bool B1_CH0_onState;
            bool B1_CH1_onState;
            bool B1_CH2_onState;
            bool B1_CH3_onState;
        };

        static constexpr CompileTimeArray<eMsgContentID, TO_UNDERLYING(eMsgContentID::eLAST)> VALID_MSG_IDS
            = {eMsgContentID::B_0_CH_2_ON_STATE,
               eMsgContentID::B_0_CH_3_ON_STATE,
               eMsgContentID::B_1_CH_0_ON_STATE,
               eMsgContentID::B_1_CH_1_ON_STATE,
               eMsgContentID::B_1_CH_2_ON_STATE,
               eMsgContentID::B_1_CH_3_ON_STATE};

      public:
        DdbCmd():
            Msg(Constant::eMsgId::DDB_CMD)
        {
            _data.B0_CH2_onState = static_cast<decltype(_data.B0_CH2_onState)>(0);
            _data.B0_CH3_onState = static_cast<decltype(_data.B0_CH3_onState)>(0);
            _data.B1_CH0_onState = static_cast<decltype(_data.B1_CH0_onState)>(0);
            _data.B1_CH1_onState = static_cast<decltype(_data.B1_CH1_onState)>(0);
            _data.B1_CH2_onState = static_cast<decltype(_data.B1_CH2_onState)>(0);
            _data.B1_CH3_onState = static_cast<decltype(_data.B1_CH3_onState)>(0);
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
                LOG_DEBUG(Logger::Nodes::DdbCmd_msg,
                          "Mismatch between received message and local message definition. Received msgContentId: (%u), "
                          "expected lower than (%u) and none zero",
                          TO_UNDERLYING(msgContentId),
                          TO_UNDERLYING(eMsgContentID::eLAST));
                return eLoadMsgCode::ERROR_MISMATCH;
            }

            bool success = false;
            switch (msgContentId)
            {
                case eMsgContentID::B_0_CH_2_ON_STATE:
                    success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.B0_CH2_onState);
                    LOG_DEBUG(Logger::Nodes::DdbCmd_msg,
                              "switch (msgContentId) case eMsgContentID::B_0_CH_2_ON_STATE: %s",
                              success ? "success" : "failed");
                    break;

                case eMsgContentID::B_0_CH_3_ON_STATE:
                    success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.B0_CH3_onState);
                    LOG_DEBUG(Logger::Nodes::DdbCmd_msg,
                              "switch (msgContentId) case eMsgContentID::B_0_CH_3_ON_STATE: %s",
                              success ? "success" : "failed");
                    break;

                case eMsgContentID::B_1_CH_0_ON_STATE:
                    success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.B1_CH0_onState);
                    LOG_DEBUG(Logger::Nodes::DdbCmd_msg,
                              "switch (msgContentId) case eMsgContentID::B_1_CH_0_ON_STATE: %s",
                              success ? "success" : "failed");
                    break;

                case eMsgContentID::B_1_CH_1_ON_STATE:
                    success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.B1_CH1_onState);
                    LOG_DEBUG(Logger::Nodes::DdbCmd_msg,
                              "switch (msgContentId) case eMsgContentID::B_1_CH_1_ON_STATE: %s",
                              success ? "success" : "failed");
                    break;

                case eMsgContentID::B_1_CH_2_ON_STATE:
                    success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.B1_CH2_onState);
                    LOG_DEBUG(Logger::Nodes::DdbCmd_msg,
                              "switch (msgContentId) case eMsgContentID::B_1_CH_2_ON_STATE: %s",
                              success ? "success" : "failed");
                    break;

                case eMsgContentID::B_1_CH_3_ON_STATE:
                    success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.B1_CH3_onState);
                    LOG_DEBUG(Logger::Nodes::DdbCmd_msg,
                              "switch (msgContentId) case eMsgContentID::B_1_CH_3_ON_STATE: %s",
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
                case eMsgContentID::B_0_CH_2_ON_STATE:
                    Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.B0_CH2_onState, msg_);
                    break;

                case eMsgContentID::B_0_CH_3_ON_STATE:
                    Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.B0_CH3_onState, msg_);
                    break;

                case eMsgContentID::B_1_CH_0_ON_STATE:
                    Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.B1_CH0_onState, msg_);
                    break;

                case eMsgContentID::B_1_CH_1_ON_STATE:
                    Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.B1_CH1_onState, msg_);
                    break;

                case eMsgContentID::B_1_CH_2_ON_STATE:
                    Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.B1_CH2_onState, msg_);
                    break;

                case eMsgContentID::B_1_CH_3_ON_STATE:
                    Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.B1_CH3_onState, msg_);
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

#endif  // ROVER_CAN2_MSGS_DDB_CMD_HPP
