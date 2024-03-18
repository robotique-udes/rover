#ifndef __PROPULSION_MOTOR_HPP__
#define __PROPULSION_MOTOR_HPP__

#include <cstdint>
#include "rover_can_lib/msgs/msg.hpp"

#if defined(ESP32)
#include "driver/twai.h"
#elif defined(__linux__) // defined(ESP32)
#include <linux/can.h>
#endif // defined(ESP32)

namespace RoverCanLib
{
    namespace Helpers
    {
#if defined(ESP32)
        twai_message_t getErrorIdMsg(void);

        template <typename COPY_TYPE, typename UNION_TYPE>
        void canMsgToStruct(IN const twai_message_t *msg_, OUT COPY_TYPE *dest_);

        template <typename COPY_TYPE, typename UNION_TYPE>
        void structToCanMsg(IN const COPY_TYPE *structMember_, OUT twai_message_t *msg_);
#endif
    }
}

#include "rover_can_lib/helpers.hpp"

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
            bool enable;
            float targetSpeed;
            float currentSpeed;
            float kp;
            float ki;
            float kd;
            bool closeLoop;
        };

        PropulsionMotor() 
        {
            data.enable = false;
            data.targetSpeed = 0.0f;
            data.currentSpeed = 0.0f;
            data.kp = 0.0f;
            data.ki = 0.0f;
            data.kd = 0.0f;
            data.closeLoop = false;
        }
        ~PropulsionMotor() {}

#if defined(ESP32)
        Constant::eInternalErrorCode parseMsg(const twai_message_t *msg_)
        {
            if (msg_->data[(uint8_t)Constant::eDataIndex::MSG_ID] != (uint8_t)Constant::eMsgId::PROPULSION_MOTOR)
            {
                LOG(ERROR, "Mismatch in message types, maybe the lib version isn't the same between all nodes... Dropping msg");
                return Constant::eInternalErrorCode::WARNING;
            }

            switch ((Msgs::PropulsionMotor::eMsgID)(msg_->data[(uint8_t)Constant::eDataIndex::MSG_CONTENT_ID]))
            {
            case eMsgID::ENABLE:
                RoverCanLib::Helpers::canMsgToStruct<bool, UnionDefinition::BoolUnion>(msg_, &this->data.enable);
                break;

            case eMsgID::TARGET_SPEED:
                RoverCanLib::Helpers::canMsgToStruct<float, UnionDefinition::FloatUnion>(msg_, &this->data.targetSpeed);
                break;

            case eMsgID::CURRENT_SPEED:
                RoverCanLib::Helpers::canMsgToStruct<float, UnionDefinition::FloatUnion>(msg_, &this->data.currentSpeed);
                break;

            case eMsgID::KP:
                RoverCanLib::Helpers::canMsgToStruct<float, UnionDefinition::FloatUnion>(msg_, &this->data.kp);
                break;

            case eMsgID::KI:
                RoverCanLib::Helpers::canMsgToStruct<float, UnionDefinition::FloatUnion>(msg_, &this->data.ki);
                break;

            case eMsgID::KD:
                RoverCanLib::Helpers::canMsgToStruct<float, UnionDefinition::FloatUnion>(msg_, &this->data.kd);
                break;

            case eMsgID::CLOSE_LOOP:
                RoverCanLib::Helpers::canMsgToStruct<bool, UnionDefinition::BoolUnion>(msg_, &this->data.closeLoop);
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

            case eMsgID::KI:
                Helpers::structToCanMsg<float, UnionDefinition::FloatUnion>(&data.ki, msg_);
                break;

            case eMsgID::KD:
                Helpers::structToCanMsg<float, UnionDefinition::FloatUnion>(&data.kd, msg_);
                break;

            case eMsgID::CLOSE_LOOP:
                Helpers::structToCanMsg<bool, UnionDefinition::BoolUnion>(&data.closeLoop, msg_);
                break;

            default:
                LOG(ERROR, "Shouldn't ever fall here, implementation error");
                *msg_ = RoverCanLib::Helpers::getErrorIdMsg();
                return Constant::eInternalErrorCode::ERROR;
            }

            return Constant::eInternalErrorCode::OK;
        }
#elif defined(__linux__) // defined(ESP32)
        Constant::eInternalErrorCode parseMsg(const can_frame *msg_, rclcpp::Logger logger_)
        {
            if (msg_->data[(uint8_t)Constant::eDataIndex::MSG_ID] != (uint8_t)Constant::eMsgId::PROPULSION_MOTOR)
            {
                RCLCPP_ERROR(logger_, "Mismatch in message types, maybe the lib version isn't the same between all nodes... Dropping msg");
                return Constant::eInternalErrorCode::WARNING;
            }

            switch ((Msgs::PropulsionMotor::eMsgID)(msg_->data[(uint8_t)Constant::eDataIndex::MSG_CONTENT_ID]))
            {
            case eMsgID::ENABLE:
                RoverCanLib::Helpers::canMsgToStruct<bool, UnionDefinition::BoolUnion>(msg_, &this->data.enable, logger_);
                break;

            case eMsgID::TARGET_SPEED:
                RoverCanLib::Helpers::canMsgToStruct<float, UnionDefinition::FloatUnion>(msg_, &this->data.targetSpeed, logger_);
                break;

            case eMsgID::CURRENT_SPEED:
                RoverCanLib::Helpers::canMsgToStruct<float, UnionDefinition::FloatUnion>(msg_, &this->data.currentSpeed, logger_);
                break;

            case eMsgID::KP:
                RoverCanLib::Helpers::canMsgToStruct<float, UnionDefinition::FloatUnion>(msg_, &this->data.kp, logger_);
                break;

            case eMsgID::KI:
                RoverCanLib::Helpers::canMsgToStruct<float, UnionDefinition::FloatUnion>(msg_, &this->data.ki, logger_);
                break;

            case eMsgID::KD:
                RoverCanLib::Helpers::canMsgToStruct<float, UnionDefinition::FloatUnion>(msg_, &this->data.kd, logger_);
                break;

            case eMsgID::CLOSE_LOOP:
                RoverCanLib::Helpers::canMsgToStruct<bool, UnionDefinition::BoolUnion>(msg_, &this->data.closeLoop, logger_);
                break;

            default:
                RCLCPP_WARN(logger_, "Unknown \"Message Specific Id\"");
                return Constant::eInternalErrorCode::ERROR;
            }

            return Constant::eInternalErrorCode::OK;
        }

        Constant::eInternalErrorCode getMsg(IN uint8_t msgId_, OUT can_frame *msg_, rclcpp::Logger logger_)
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

            case eMsgID::KI:
                Helpers::structToCanMsg<float, UnionDefinition::FloatUnion>(&data.ki, msg_);
                break;

            case eMsgID::KD:
                Helpers::structToCanMsg<float, UnionDefinition::FloatUnion>(&data.kd, msg_);
                break;

            case eMsgID::CLOSE_LOOP:
                Helpers::structToCanMsg<bool, UnionDefinition::BoolUnion>(&data.closeLoop, msg_);
                break;

            default:
                RCLCPP_ERROR(logger_, "Shouldn't ever fall here, implementation error");
                *msg_ = RoverCanLib::Helpers::getErrorIdMsg();
                return Constant::eInternalErrorCode::ERROR;
            }

            return Constant::eInternalErrorCode::OK;
        }
#endif // defined(ESP32)

        uint8_t getMsgIDNb(void)
        {
            return (uint8_t)eMsgID::eLAST;
        }

        sMsgData data;
    };
}

#endif // __PROPULSION_MOTOR_HPP__
