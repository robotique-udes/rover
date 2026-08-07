#ifndef SCIENCE_INFO_HPP
#define SCIENCE_INFO_HPP

#include "rover_can2/msgs/msg.hpp"
#include "rover_can2/helpers.hpp"


DEFINE_LOG_NODE(ScienceInfo_msg, Logger::eNodeState::OFF)

namespace RoverCan2::Msgs
{
    class ScienceInfo : public Msg<ScienceInfo>
    {
      public:
        enum class eMsgContentID : uint8_t
        {
            SAMPLE_INDEX,
            SENSOR_1,
            SENSOR_2,
            SENSOR_3,
            eLAST,
        };

      private:
        struct sMsgData
        {
            uint32_t sample_index;
            int sensor_1;
            int sensor_2;
            int sensor_3;

            static_assert(sizeof(sample_index) <= RoverCan2::Constant::CAN_MAX_DATA_LENGTH - TO_UNDERLYING(RoverCan2::Constant::eDataIndex::START_OF_DATA), "Can messages cannot include field longer than 6 bytes");
            static_assert(sizeof(sensor_1) <= RoverCan2::Constant::CAN_MAX_DATA_LENGTH - TO_UNDERLYING(RoverCan2::Constant::eDataIndex::START_OF_DATA), "Can messages cannot include field longer than 6 bytes");
            static_assert(sizeof(sensor_2) <= RoverCan2::Constant::CAN_MAX_DATA_LENGTH - TO_UNDERLYING(RoverCan2::Constant::eDataIndex::START_OF_DATA), "Can messages cannot include field longer than 6 bytes");
            static_assert(sizeof(sensor_3) <= RoverCan2::Constant::CAN_MAX_DATA_LENGTH - TO_UNDERLYING(RoverCan2::Constant::eDataIndex::START_OF_DATA), "Can messages cannot include field longer than 6 bytes");
        };

        static constexpr CompileTimeArray<eMsgContentID, TO_UNDERLYING(eMsgContentID::eLAST)> VALID_MSG_IDS
            = {eMsgContentID::SAMPLE_INDEX, eMsgContentID::SENSOR_1, eMsgContentID::SENSOR_2, eMsgContentID::SENSOR_3};

      public:
        ScienceInfo():
            Msg(Constant::eMsgId::SCIENCE_INFO)
        {
            _data.sample_index = static_cast<decltype(_data.sample_index)>(0);
            _data.sensor_1 = static_cast<decltype(_data.sensor_1)>(0);
            _data.sensor_2 = static_cast<decltype(_data.sensor_2)>(0);
            _data.sensor_3 = static_cast<decltype(_data.sensor_3)>(0);
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
                LOG_DEBUG(Logger::Nodes::ScienceInfo_msg,
                          "Mismatch between received message and local message definition. Received msgContentId: (%u), "
                          "expected lower than (%u) and none zero",
                          TO_UNDERLYING(msgContentId),
                          TO_UNDERLYING(eMsgContentID::eLAST));
                return eLoadMsgCode::ERROR_MISMATCH;
            }

            bool success = false;
            switch (msgContentId)
            {
                case eMsgContentID::SAMPLE_INDEX:
                    success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.sample_index);
                    LOG_DEBUG(Logger::Nodes::ScienceInfo_msg,
                              "switch (msgContentId) case eMsgContentID::SAMPLE_INDEX: %s",
                              success ? "success" : "failed");
                    break;

                case eMsgContentID::SENSOR_1:
                    success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.sensor_1);
                    LOG_DEBUG(Logger::Nodes::ScienceInfo_msg,
                              "switch (msgContentId) case eMsgContentID::SENSOR_1: %s",
                              success ? "success" : "failed");
                    break;

                case eMsgContentID::SENSOR_2:
                    success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.sensor_2);
                    LOG_DEBUG(Logger::Nodes::ScienceInfo_msg,
                              "switch (msgContentId) case eMsgContentID::SENSOR_2: %s",
                              success ? "success" : "failed");
                    break;

                case eMsgContentID::SENSOR_3:
                    success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.sensor_3);
                    LOG_DEBUG(Logger::Nodes::ScienceInfo_msg,
                              "switch (msgContentId) case eMsgContentID::SENSOR_3: %s",
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
                case eMsgContentID::SAMPLE_INDEX:
                    Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.sample_index, msg_);
                    break;

                case eMsgContentID::SENSOR_1:
                    Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.sensor_1, msg_);
                    break;

                case eMsgContentID::SENSOR_2:
                    Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.sensor_2, msg_);
                    break;

                case eMsgContentID::SENSOR_3:
                    Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.sensor_3, msg_);
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

#endif  // SCIENCE_INFO_HPP
