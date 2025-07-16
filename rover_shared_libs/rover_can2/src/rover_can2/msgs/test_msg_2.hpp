#ifndef ROVER_CAN2_MSGS_TEST_MSG_2_HPP
#define ROVER_CAN2_MSGS_TEST_MSG_2_HPP

#include "rover_can2/msgs/msg.hpp"
#include "rover_can2/helpers.hpp"

DEFINE_LOG_NODE(TestMsg2_msg, Logger::eNodeState::OFF)

namespace RoverCan2::Msgs
{
    class TestMsg2 : public Msg<TestMsg2>
    {
      public:
        enum class eMsgContentID : uint8_t
        {
            CMD,
            CLOSE_LOOP,
            eLAST,
        };

      private:
        struct sMsgData
        {
            float cmd;
            bool closeLoop;
        };

        static constexpr CompileTimeArray<eMsgContentID, TO_UNDERLYING(eMsgContentID::eLAST)> VALID_MSG_IDS
            = {eMsgContentID::CMD, eMsgContentID::CLOSE_LOOP};

      public:
        TestMsg2():
            Msg(Constant::eMsgId::TEST_MSG_2)
        {
            _data.cmd = static_cast<decltype(_data.cmd)>(0);
            _data.closeLoop = static_cast<decltype(_data.closeLoop)>(0);
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
                LOG_DEBUG(Logger::Nodes::TestMsg2_msg,
                          "Mismatch between received message and local message definition. Received msgContentId: (%u), "
                          "expected lower than (%u) and none zero",
                          TO_UNDERLYING(msgContentId),
                          TO_UNDERLYING(eMsgContentID::eLAST));
                return eLoadMsgCode::ERROR_MISMATCH;
            }

            bool success = false;
            switch (msgContentId)
            {
                case eMsgContentID::CMD:
                    success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.cmd);
                    LOG_DEBUG(Logger::Nodes::TestMsg2_msg,
                              "switch (msgContentId) case eMsgContentID::CMD: %s",
                              success ? "success" : "failed");
                    break;

                case eMsgContentID::CLOSE_LOOP:
                    success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.closeLoop);
                    LOG_DEBUG(Logger::Nodes::TestMsg2_msg,
                              "switch (msgContentId) case eMsgContentID::CLOSE_LOOP: %s",
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
                case eMsgContentID::CMD:
                    Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.cmd, msg_);
                    break;

                case eMsgContentID::CLOSE_LOOP:
                    Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.closeLoop, msg_);
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

#endif  // ROVER_CAN2_MSGS_TEST_MSG_2_HPP
