#ifndef ROVER_CAN2_MSGS_SENSOR_BOX_HPP
#define ROVER_CAN2_MSGS_SENSOR_BOX_HPP

#include "rover_can2/msgs/msg.hpp"
#include "rover_can2/helpers.hpp"

DEFINE_LOG_NODE(SensorBox_msg, Logger::eNodeState::OFF)

namespace RoverCan2::Msgs
{
    class SensorBox : public Msg<SensorBox>
    {
      public:
        enum class eMsgContentID : uint8_t
        {
            AMONIA,
            HYDROGEN,
            eLAST,
        };

      private:
        struct sMsgData
        {
            float amonia;
            float hydrogen;

            static_assert(sizeof(amonia) <= RoverCan2::Constant::CAN_MAX_DATA_LENGTH
                                                - TO_UNDERLYING(RoverCan2::Constant::eDataIndex::START_OF_DATA),
                          "Can messages cannot include field longer than 6 bytes");
            static_assert(sizeof(hydrogen) <= RoverCan2::Constant::CAN_MAX_DATA_LENGTH
                                                  - TO_UNDERLYING(RoverCan2::Constant::eDataIndex::START_OF_DATA),
                          "Can messages cannot include field longer than 6 bytes");
        };

        static constexpr CompileTimeArray<eMsgContentID, TO_UNDERLYING(eMsgContentID::eLAST)> VALID_MSG_IDS
            = {eMsgContentID::AMONIA, eMsgContentID::HYDROGEN};

      public:
        SensorBox():
            Msg(Constant::eMsgId::SENSOR_BOX)
        {
            _data.amonia = static_cast<decltype(_data.amonia)>(0);
            _data.hydrogen = static_cast<decltype(_data.hydrogen)>(0);
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
                LOG_DEBUG(Logger::Nodes::SensorBox_msg,
                          "Mismatch between received message and local message definition. Received msgContentId: (%u), "
                          "expected lower than (%u) and none zero",
                          TO_UNDERLYING(msgContentId),
                          TO_UNDERLYING(eMsgContentID::eLAST));
                return eLoadMsgCode::ERROR_MISMATCH;
            }

            bool success = false;
            switch (msgContentId)
            {
                case eMsgContentID::AMONIA:
                    success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.amonia);
                    LOG_DEBUG(Logger::Nodes::SensorBox_msg,
                              "switch (msgContentId) case eMsgContentID::AMONIA: %s",
                              success ? "success" : "failed");
                    break;

                case eMsgContentID::HYDROGEN:
                    success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.hydrogen);
                    LOG_DEBUG(Logger::Nodes::SensorBox_msg,
                              "switch (msgContentId) case eMsgContentID::HYDROGEN: %s",
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
                case eMsgContentID::AMONIA:
                    Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.amonia, msg_);
                    break;

                case eMsgContentID::HYDROGEN:
                    Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.hydrogen, msg_);
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

#endif  // ROVER_CAN2_MSGS_SENSOR_BOX_HPP
