#ifndef ARM_JOINT_ADVANCED_STATUS_HPP
#define ARM_JOINT_ADVANCED_STATUS_HPP

#include "rover_can2/msgs/msg.hpp"
#include "rover_can2/helpers.hpp"


DEFINE_LOG_NODE(ArmJointAdvancedStatus_msg, Logger::eNodeState::OFF)

namespace RoverCan2::Msgs
{
    class ArmJointAdvancedStatus : public Msg<ArmJointAdvancedStatus>
    {
      public:
        enum class eMsgContentID : uint8_t
        {
            CURRENT_TORQUE,
            CURRENT_MOTOR_TEMP,
            CURRENT_AMPERAGE,
            eLAST,
        };

      private:
        struct sMsgData
        {
            float currentTorque;
            float currentMotorTemp;
            float currentAmperage;

            static_assert(sizeof(currentTorque) <= RoverCan2::Constant::CAN_MAX_DATA_LENGTH - TO_UNDERLYING(RoverCan2::Constant::eDataIndex::START_OF_DATA), "Can messages cannot include field longer than 6 bytes");
            static_assert(sizeof(currentMotorTemp) <= RoverCan2::Constant::CAN_MAX_DATA_LENGTH - TO_UNDERLYING(RoverCan2::Constant::eDataIndex::START_OF_DATA), "Can messages cannot include field longer than 6 bytes");
            static_assert(sizeof(currentAmperage) <= RoverCan2::Constant::CAN_MAX_DATA_LENGTH - TO_UNDERLYING(RoverCan2::Constant::eDataIndex::START_OF_DATA), "Can messages cannot include field longer than 6 bytes");
        };

        static constexpr CompileTimeArray<eMsgContentID, TO_UNDERLYING(eMsgContentID::eLAST)> VALID_MSG_IDS
            = {eMsgContentID::CURRENT_TORQUE, eMsgContentID::CURRENT_MOTOR_TEMP, eMsgContentID::CURRENT_AMPERAGE};

      public:
        ArmJointAdvancedStatus():
            Msg(Constant::eMsgId::ARM_JOINT_ADVANCED_STATUS)
        {
            _data.currentTorque = static_cast<decltype(_data.currentTorque)>(0);
            _data.currentMotorTemp = static_cast<decltype(_data.currentMotorTemp)>(0);
            _data.currentAmperage = static_cast<decltype(_data.currentAmperage)>(0);
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
                LOG_DEBUG(Logger::Nodes::ArmJointAdvancedStatus_msg,
                          "Mismatch between received message and local message definition. Received msgContentId: (%u), "
                          "expected lower than (%u) and none zero",
                          TO_UNDERLYING(msgContentId),
                          TO_UNDERLYING(eMsgContentID::eLAST));
                return eLoadMsgCode::ERROR_MISMATCH;
            }

            bool success = false;
            switch (msgContentId)
            {
                case eMsgContentID::CURRENT_TORQUE:
                    success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.currentTorque);
                    LOG_DEBUG(Logger::Nodes::ArmJointAdvancedStatus_msg,
                              "switch (msgContentId) case eMsgContentID::CURRENT_TORQUE: %s",
                              success ? "success" : "failed");
                    break;

                case eMsgContentID::CURRENT_MOTOR_TEMP:
                    success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.currentMotorTemp);
                    LOG_DEBUG(Logger::Nodes::ArmJointAdvancedStatus_msg,
                              "switch (msgContentId) case eMsgContentID::CURRENT_MOTOR_TEMP: %s",
                              success ? "success" : "failed");
                    break;

                case eMsgContentID::CURRENT_AMPERAGE:
                    success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.currentAmperage);
                    LOG_DEBUG(Logger::Nodes::ArmJointAdvancedStatus_msg,
                              "switch (msgContentId) case eMsgContentID::CURRENT_AMPERAGE: %s",
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
                case eMsgContentID::CURRENT_TORQUE:
                    Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.currentTorque, msg_);
                    break;

                case eMsgContentID::CURRENT_MOTOR_TEMP:
                    Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.currentMotorTemp, msg_);
                    break;

                case eMsgContentID::CURRENT_AMPERAGE:
                    Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.currentAmperage, msg_);
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

#endif  // ARM_JOINT_ADVANCED_STATUS_HPP
