#ifndef MORSE_CODE_HPP
#define MORSE_CODE_HPP

#include "rover_can2/msgs/msg.hpp"
#include "rover_can2/helpers.hpp"

DEFINE_LOG_NODE(MorseCode_msg, Logger::eNodeState::OFF)

namespace RoverCan2::Msgs
{
    class MorseCode : public Msg<MorseCode>
    {
      public:
        enum class eMsgContentID : uint8_t
        {
            START,
            INDEX,
            CHARACTER,
            MSG_LENGTH,
            CHECKSUM,
            eLAST,
        };

      private:
        struct sMsgData
        {
            bool start;
            uint8_t index;
            uint8_t character;
            uint8_t msg_length;
            uint8_t checksum;

            static_assert(sizeof(start) <= RoverCan2::Constant::CAN_MAX_DATA_LENGTH
                                               - TO_UNDERLYING(RoverCan2::Constant::eDataIndex::START_OF_DATA),
                          "Can messages cannot include field longer than 6 bytes");
            static_assert(sizeof(index) <= RoverCan2::Constant::CAN_MAX_DATA_LENGTH
                                               - TO_UNDERLYING(RoverCan2::Constant::eDataIndex::START_OF_DATA),
                          "Can messages cannot include field longer than 6 bytes");
            static_assert(sizeof(character) <= RoverCan2::Constant::CAN_MAX_DATA_LENGTH
                                                   - TO_UNDERLYING(RoverCan2::Constant::eDataIndex::START_OF_DATA),
                          "Can messages cannot include field longer than 6 bytes");
            static_assert(sizeof(msg_length) <= RoverCan2::Constant::CAN_MAX_DATA_LENGTH
                                                    - TO_UNDERLYING(RoverCan2::Constant::eDataIndex::START_OF_DATA),
                          "Can messages cannot include field longer than 6 bytes");
            static_assert(sizeof(checksum) <= RoverCan2::Constant::CAN_MAX_DATA_LENGTH
                                                  - TO_UNDERLYING(RoverCan2::Constant::eDataIndex::START_OF_DATA),
                          "Can messages cannot include field longer than 6 bytes");
        };

        static constexpr CompileTimeArray<eMsgContentID, TO_UNDERLYING(eMsgContentID::eLAST)> VALID_MSG_IDS
            = {eMsgContentID::START,
               eMsgContentID::INDEX,
               eMsgContentID::CHARACTER,
               eMsgContentID::MSG_LENGTH,
               eMsgContentID::CHECKSUM};

      public:
        MorseCode():
            Msg(Constant::eMsgId::MORSE_CODE)
        {
            _data.start = static_cast<decltype(_data.start)>(0);
            _data.index = static_cast<decltype(_data.index)>(0);
            _data.character = static_cast<decltype(_data.character)>(0);
            _data.msg_length = static_cast<decltype(_data.msg_length)>(0);
            _data.checksum = static_cast<decltype(_data.checksum)>(0);
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
                LOG_DEBUG(Logger::Nodes::MorseCode_msg,
                          "Mismatch between received message and local message definition. Received msgContentId: (%u), "
                          "expected lower than (%u) and none zero",
                          TO_UNDERLYING(msgContentId),
                          TO_UNDERLYING(eMsgContentID::eLAST));
                return eLoadMsgCode::ERROR_MISMATCH;
            }

            bool success = false;
            switch (msgContentId)
            {
                case eMsgContentID::START:
                    success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.start);
                    LOG_DEBUG(Logger::Nodes::MorseCode_msg,
                              "switch (msgContentId) case eMsgContentID::START: %s",
                              success ? "success" : "failed");
                    break;

                case eMsgContentID::INDEX:
                    success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.index);
                    LOG_DEBUG(Logger::Nodes::MorseCode_msg,
                              "switch (msgContentId) case eMsgContentID::INDEX: %s",
                              success ? "success" : "failed");
                    break;

                case eMsgContentID::CHARACTER:
                    success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.character);
                    LOG_DEBUG(Logger::Nodes::MorseCode_msg,
                              "switch (msgContentId) case eMsgContentID::CHARACTER: %s",
                              success ? "success" : "failed");
                    break;

                case eMsgContentID::MSG_LENGTH:
                    success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.msg_length);
                    LOG_DEBUG(Logger::Nodes::MorseCode_msg,
                              "switch (msgContentId) case eMsgContentID::MSG_LENGTH: %s",
                              success ? "success" : "failed");
                    break;

                case eMsgContentID::CHECKSUM:
                    success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.checksum);
                    LOG_DEBUG(Logger::Nodes::MorseCode_msg,
                              "switch (msgContentId) case eMsgContentID::CHECKSUM: %s",
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
                case eMsgContentID::START:
                    Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.start, msg_);
                    break;

                case eMsgContentID::INDEX:
                    Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.index, msg_);
                    break;

                case eMsgContentID::CHARACTER:
                    Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.character, msg_);
                    break;

                case eMsgContentID::MSG_LENGTH:
                    Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.msg_length, msg_);
                    break;

                case eMsgContentID::CHECKSUM:
                    Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.checksum, msg_);
                    break;
                case eMsgContentID::eLAST:
                    [[fallthrough]];
                default:
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

#endif  // MORSE_CODE_HPP
