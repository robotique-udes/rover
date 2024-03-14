#ifndef __PROPULSION_MOTOR_HPP__
#define __PROPULSION_MOTOR_HPP__

#include <cstdint>
// #include "rover_can_lib/rover_can_lib.hpp"

namespace RoverCanLib::Msgs
{
    class PropulsionMotor : public Msg
    {
    public:
        enum class eMsgID : uint8_t
        {
            NOT_USED = 0x00,
            ENABLE = 0x01,
            TARGET_SPEED = 0x02,
            CURRENT_SPEED = 0x03,
            KP = 0x04,
            KI = 0x05,
            KD = 0x06,
            CLOSE_LOOP = 0x07,
            eLAST
        };

        struct sMsgData
        {
            bool enable = false;
            float targetSpeed = 0.0f;
            float currentSpeed = 0.0f;
            float kp = 0.0f;
            float ki = 0.0f;
            float kd = 0.0f;
            bool closeLoop = false;
        };

        PropulsionMotor() {}
        ~PropulsionMotor() {}

        Constant::eInternalErrorCode parseMsg(const twai_message_t *msg_)
        {
            if (msg_->data[(uint8_t)Constant::eDataIndex::MSG_ID] != (uint8_t)Constant::eMsgId::PROPULSION_MOTOR)
            {
                LOG(ERROR,
                    "Mismatch in message types. "
                    "Received type: %u but expected %u, maybe the lib version isn't the same between all nodes... "
                    "Dropping msg",
                    msg_->data[(uint8_t)Constant::eDataIndex::MSG_ID],
                    (uint8_t)Constant::eMsgId::PROPULSION_MOTOR);
                return Constant::eInternalErrorCode::WARNING;
            }

            switch ((Msgs::PropulsionMotor::eMsgID)(msg_->data[(uint8_t)Constant::eDataIndex::MSG_CONTENT_ID]))
            {
            case Msgs::PropulsionMotor::eMsgID::ENABLE:
                Helpers::canMsgToStruct<bool, UnionDefinition::BoolUnion>(msg_, &this->data.enable);
                break;

            case Msgs::PropulsionMotor::eMsgID::TARGET_SPEED:

                Helpers::canMsgToStruct<float, UnionDefinition::FloatUnion>(msg_, &this->data.targetSpeed);
                break;

            case Msgs::PropulsionMotor::eMsgID::KP:
                Helpers::canMsgToStruct<float, UnionDefinition::FloatUnion>(msg_, &this->data.kp);
                break;

            case Msgs::PropulsionMotor::eMsgID::KI:
                Helpers::canMsgToStruct<float, UnionDefinition::FloatUnion>(msg_, &this->data.ki);
                break;

            case Msgs::PropulsionMotor::eMsgID::KD:
                Helpers::canMsgToStruct<float, UnionDefinition::FloatUnion>(msg_, &this->data.kd);
                break;

            case Msgs::PropulsionMotor::eMsgID::CLOSE_LOOP:
                Helpers::canMsgToStruct<bool, UnionDefinition::BoolUnion>(msg_, &this->data.closeLoop);
                break;

            default:
                LOG(WARN, "Unknown \"Message Specific Id\"");
                return Constant::eInternalErrorCode::ERROR;
            }

            return Constant::eInternalErrorCode::OK;
        }

        Constant::eInternalErrorCode getMsg(IN uint8_t msgId_, OUT twai_message_t *msg_)
        {
            msg_->data[(uint8_t)Constant::eDataIndex::MSG_ID] = (uint8_t)Constant::eMsgId::PROPULSION_MOTOR;
            msg_->data[(uint8_t)Constant::eDataIndex::MSG_CONTENT_ID] = msgId_;

            switch ((RoverCanLib::Msgs::PropulsionMotor::eMsgID)msgId_)
            {
            case eMsgID::ENABLE:
                Helpers::structToCanMsg<bool, UnionDefinition::BoolUnion>(&data.enable, msg_);
                break;

            case eMsgID::TARGET_SPEED:
                Helpers::structToCanMsg<float, UnionDefinition::FloatUnion>(&data.targetSpeed, msg_);
                break;

            case eMsgID::CURRENT_SPEED:
                Helpers::structToCanMsg<float, UnionDefinition::FloatUnion>(&data.currentSpeed, msg_);
                break;

            case eMsgID::KP:
                Helpers::structToCanMsg<float, UnionDefinition::FloatUnion>(&data.kp, msg_);
                break;

            case eMsgID::KD:
                Helpers::structToCanMsg<float, UnionDefinition::FloatUnion>(&data.kd, msg_);
                break;

            case eMsgID::KI:
                Helpers::structToCanMsg<float, UnionDefinition::FloatUnion>(&data.ki, msg_);
                break;

            case eMsgID::CLOSE_LOOP:
                Helpers::structToCanMsg<bool, UnionDefinition::BoolUnion>(&data.closeLoop, msg_);
                break;

            default:
                LOG(ERROR, "Shouldn't ever fall here, implementation error");
                *msg_ = RoverCanLib::Helpers::getErrorIdMsg();
                Constant::eInternalErrorCode::ERROR;
            }

            return Constant::eInternalErrorCode::OK;
        }

        uint8_t getMsgIDNb(void)
        {
            return (uint8_t)eMsgID::eLAST;
        }

        sMsgData data;
    };
}

#endif // __PROPULSION_MOTOR_HPP__
