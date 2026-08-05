#ifndef SCIENCE_CMD_HPP
#define SCIENCE_CMD_HPP

#include "rover_can2/msgs/msg.hpp"
#include "rover_can2/helpers.hpp"

DEFINE_LOG_NODE(ScienceCmd_msg, Logger::eNodeState::OFF)

namespace RoverCan2::Msgs
{
    class ScienceCmd : public Msg<ScienceCmd>
    {
      public:
        enum class eMsgContentID : uint8_t
        {
            LIN_ACT_SPEED,
            GRINDER_ON,
            BEAK_POS,
            CARROUSEL_ON,
            eLAST,
        };

      private:
        struct sMsgData
        {
            float lin_act_speed;
            bool grinder_on;
            float beak_pos;
            bool carrousel_on;

            static_assert(sizeof(lin_act_speed) <= RoverCan2::Constant::CAN_MAX_DATA_LENGTH
                                                       - TO_UNDERLYING(RoverCan2::Constant::eDataIndex::START_OF_DATA),
                          "Can messages cannot include field longer than 6 bytes");
            static_assert(sizeof(grinder_on) <= RoverCan2::Constant::CAN_MAX_DATA_LENGTH
                                                    - TO_UNDERLYING(RoverCan2::Constant::eDataIndex::START_OF_DATA),
                          "Can messages cannot include field longer than 6 bytes");
            static_assert(sizeof(beak_pos) <= RoverCan2::Constant::CAN_MAX_DATA_LENGTH
                                                  - TO_UNDERLYING(RoverCan2::Constant::eDataIndex::START_OF_DATA),
                          "Can messages cannot include field longer than 6 bytes");
            static_assert(sizeof(carrousel_on) <= RoverCan2::Constant::CAN_MAX_DATA_LENGTH
                                                      - TO_UNDERLYING(RoverCan2::Constant::eDataIndex::START_OF_DATA),
                          "Can messages cannot include field longer than 6 bytes");
        };

        static constexpr CompileTimeArray<eMsgContentID, TO_UNDERLYING(eMsgContentID::eLAST)> VALID_MSG_IDS
            = {eMsgContentID::LIN_ACT_SPEED, eMsgContentID::GRINDER_ON, eMsgContentID::BEAK_POS, eMsgContentID::CARROUSEL_ON};

      public:
        ScienceCmd():
            Msg(Constant::eMsgId::SCIENCE_CMD)
        {
            _data.lin_act_speed = static_cast<decltype(_data.lin_act_speed)>(0);
            _data.grinder_on = static_cast<decltype(_data.grinder_on)>(0);
            _data.beak_pos = static_cast<decltype(_data.beak_pos)>(0);
            _data.carrousel_on = static_cast<decltype(_data.carrousel_on)>(0);
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
                LOG_DEBUG(Logger::Nodes::ScienceCmd_msg,
                          "Mismatch between received message and local message definition. Received msgContentId: (%u), "
                          "expected lower than (%u) and none zero",
                          TO_UNDERLYING(msgContentId),
                          TO_UNDERLYING(eMsgContentID::eLAST));
                return eLoadMsgCode::ERROR_MISMATCH;
            }

            bool success = false;
            switch (msgContentId)
            {
                case eMsgContentID::LIN_ACT_SPEED:
                    success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.lin_act_speed);
                    LOG_DEBUG(Logger::Nodes::ScienceCmd_msg,
                              "switch (msgContentId) case eMsgContentID::LIN_ACT_SPEED: %s",
                              success ? "success" : "failed");
                    break;

                case eMsgContentID::GRINDER_ON:
                    success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.grinder_on);
                    LOG_DEBUG(Logger::Nodes::ScienceCmd_msg,
                              "switch (msgContentId) case eMsgContentID::GRINDER_ON: %s",
                              success ? "success" : "failed");
                    break;

                case eMsgContentID::BEAK_POS:
                    success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.beak_pos);
                    LOG_DEBUG(Logger::Nodes::ScienceCmd_msg,
                              "switch (msgContentId) case eMsgContentID::BEAK_POS: %s",
                              success ? "success" : "failed");
                    break;

                case eMsgContentID::CARROUSEL_ON:
                    success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.carrousel_on);
                    LOG_DEBUG(Logger::Nodes::ScienceCmd_msg,
                              "switch (msgContentId) case eMsgContentID::CARROUSEL_ON: %s",
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
                case eMsgContentID::LIN_ACT_SPEED:
                    Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.lin_act_speed, msg_);
                    break;

                case eMsgContentID::GRINDER_ON:
                    Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.grinder_on, msg_);
                    break;

                case eMsgContentID::BEAK_POS:
                    Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.beak_pos, msg_);
                    break;

                case eMsgContentID::CARROUSEL_ON:
                    Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.carrousel_on, msg_);
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

#endif  // SCIENCE_CMD_HPP
