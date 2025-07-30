#ifndef ROVER_CAN2_MSGS_ARM_JOINT_CONFIG_HPP
#define ROVER_CAN2_MSGS_ARM_JOINT_CONFIG_HPP

#include "rover_can2/msgs/msg.hpp"
#include "rover_can2/helpers.hpp"

DEFINE_LOG_NODE(ArmJointConfig_msg, Logger::eNodeState::OFF)

namespace RoverCan2::Msgs
{
    class ArmJointConfig : public Msg<ArmJointConfig>
    {
      public:
        enum class eMsgContentID : uint8_t
        {
            UPPER_LIMIT,
            LOWER_LIMIT,
            MAX_SPEED,
            KP_SPEED,
            KD_SPEED,
            KI_SPEED,
            eLAST,
        };

      private:
        struct sMsgData
        {
            float upperLimit;
            float lowerLimit;
            float maxSpeed;
            float kpSpeed;
            float kdSpeed;
            float kiSpeed;
        };

        static constexpr CompileTimeArray<eMsgContentID, TO_UNDERLYING(eMsgContentID::eLAST)> VALID_MSG_IDS
            = {eMsgContentID::UPPER_LIMIT,
               eMsgContentID::LOWER_LIMIT,
               eMsgContentID::MAX_SPEED,
               eMsgContentID::KP_SPEED,
               eMsgContentID::KD_SPEED,
               eMsgContentID::KI_SPEED};

      public:
        ArmJointConfig():
            Msg(Constant::eMsgId::ARM_JOINT_CONFIG)
        {
            _data.upperLimit = static_cast<decltype(_data.upperLimit)>(0);
            _data.lowerLimit = static_cast<decltype(_data.lowerLimit)>(0);
            _data.maxSpeed = static_cast<decltype(_data.maxSpeed)>(0);
            _data.kpSpeed = static_cast<decltype(_data.kpSpeed)>(0);
            _data.kdSpeed = static_cast<decltype(_data.kdSpeed)>(0);
            _data.kiSpeed = static_cast<decltype(_data.kiSpeed)>(0);
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
                LOG_DEBUG(Logger::Nodes::ArmJointConfig_msg,
                          "Mismatch between received message and local message definition. Received msgContentId: (%u), "
                          "expected lower than (%u) and none zero",
                          TO_UNDERLYING(msgContentId),
                          TO_UNDERLYING(eMsgContentID::eLAST));
                return eLoadMsgCode::ERROR_MISMATCH;
            }

            bool success = false;
            switch (msgContentId)
            {
                case eMsgContentID::UPPER_LIMIT:
                    success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.upperLimit);
                    LOG_DEBUG(Logger::Nodes::ArmJointConfig_msg,
                              "switch (msgContentId) case eMsgContentID::UPPER_LIMIT: %s",
                              success ? "success" : "failed");
                    break;

                case eMsgContentID::LOWER_LIMIT:
                    success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.lowerLimit);
                    LOG_DEBUG(Logger::Nodes::ArmJointConfig_msg,
                              "switch (msgContentId) case eMsgContentID::LOWER_LIMIT: %s",
                              success ? "success" : "failed");
                    break;

                case eMsgContentID::MAX_SPEED:
                    success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.maxSpeed);
                    LOG_DEBUG(Logger::Nodes::ArmJointConfig_msg,
                              "switch (msgContentId) case eMsgContentID::MAX_SPEED: %s",
                              success ? "success" : "failed");
                    break;

                case eMsgContentID::KP_SPEED:
                    success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.kpSpeed);
                    LOG_DEBUG(Logger::Nodes::ArmJointConfig_msg,
                              "switch (msgContentId) case eMsgContentID::KP_SPEED: %s",
                              success ? "success" : "failed");
                    break;

                case eMsgContentID::KD_SPEED:
                    success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.kdSpeed);
                    LOG_DEBUG(Logger::Nodes::ArmJointConfig_msg,
                              "switch (msgContentId) case eMsgContentID::KD_SPEED: %s",
                              success ? "success" : "failed");
                    break;

                case eMsgContentID::KI_SPEED:
                    success = Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, _data.kiSpeed);
                    LOG_DEBUG(Logger::Nodes::ArmJointConfig_msg,
                              "switch (msgContentId) case eMsgContentID::KI_SPEED: %s",
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
                case eMsgContentID::UPPER_LIMIT:
                    Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.upperLimit, msg_);
                    break;

                case eMsgContentID::LOWER_LIMIT:
                    Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.lowerLimit, msg_);
                    break;

                case eMsgContentID::MAX_SPEED:
                    Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.maxSpeed, msg_);
                    break;

                case eMsgContentID::KP_SPEED:
                    Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.kpSpeed, msg_);
                    break;

                case eMsgContentID::KD_SPEED:
                    Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.kdSpeed, msg_);
                    break;

                case eMsgContentID::KI_SPEED:
                    Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(this->getMsgId(), msgContentId_, _data.kiSpeed, msg_);
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

#endif  // ROVER_CAN2_MSGS_ARM_JOINT_CONFIG_HPP
