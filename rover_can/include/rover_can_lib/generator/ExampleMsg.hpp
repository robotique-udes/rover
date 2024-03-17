#ifndef __EXAMPLE_MSG_HPP__
#define __EXAMPLE_MSG_HPP__

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
    class ExampleMsg : public Msg
    {
    public:
        enum class eMsgID : uint8_t
        {
            EXAMPLE1 = 0x00,
            EXAMPLE2 = 0x01,
            EXAMPLE3 = 0x02,
            eLAST
        };

        struct sMsgData
        {
            bool example1;
            float example2;
            uint16_t example3;
        };

        ExampleMsg() {}
        ~ExampleMsg() {}

#if defined(ESP32)
        Constant::eInternalErrorCode parseMsg(const twai_message_t *msg_)
        {
            if (msg_->data[(uint8_t)Constant::eDataIndex::MSG_ID] != (uint8_t)Constant::eMsgId::ERROR_STATE)
            {
                LOG(ERROR, "Mismatch in message types, maybe the lib version isn't the same between all nodes... Dropping msg");
                return Constant::eInternalErrorCode::WARNING;
            }

            switch ((Msgs::ExampleMsg::eMsgID)(msg_->data[(uint8_t)Constant::eDataIndex::MSG_CONTENT_ID]))
            {
            case eMsgID::EXAMPLE1:
                RoverCanLib::Helpers::canMsgToStruct<bool, UnionDefinition::BoolUnion>(msg_, &this->data.example1);
                break;

            case eMsgID::EXAMPLE2:
                RoverCanLib::Helpers::canMsgToStruct<float, UnionDefinition::FloatUnion>(msg_, &this->data.example2);
                break;

            case eMsgID::EXAMPLE3:
                RoverCanLib::Helpers::canMsgToStruct<uint16_t, UnionDefinition::Uint16_tUnion>(msg_, &this->data.example3);
                break;

            default:
                LOG(WARN, "Unknown \"Message Specific Id\"");
                return Constant::eInternalErrorCode::ERROR;
            }

            return Constant::eInternalErrorCode::OK;
        }

        Constant::eInternalErrorCode getMsg(IN uint8_t msgId_, OUT twai_message_t *msg_)
        {
            msg_->data[(uint8_t)Constant::eDataIndex::MSG_ID] = (uint8_t)Constant::eMsgId::ERROR_STATE;
            msg_->data[(uint8_t)Constant::eDataIndex::MSG_CONTENT_ID] = msgId_;

            switch ((RoverCanLib::Msgs::ExampleMsg::eMsgID)msgId_)
            {
            case eMsgID::EXAMPLE1:
                Helpers::structToCanMsg<bool, UnionDefinition::BoolUnion>(&data.example1, msg_);
                break;

            case eMsgID::EXAMPLE2:
                Helpers::structToCanMsg<float, UnionDefinition::FloatUnion>(&data.example2, msg_);
                break;

            case eMsgID::EXAMPLE3:
                Helpers::structToCanMsg<uint16_t, UnionDefinition::Uint16_tUnion>(&data.example3, msg_);
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
            if (msg_->data[(uint8_t)Constant::eDataIndex::MSG_ID] != (uint8_t)Constant::eMsgId::ERROR_STATE)
            {
                RCLCPP_ERROR(logger_, "Mismatch in message types, maybe the lib version isn't the same between all nodes... Dropping msg");
                return Constant::eInternalErrorCode::WARNING;
            }

            switch ((Msgs::ExampleMsg::eMsgID)(msg_->data[(uint8_t)Constant::eDataIndex::MSG_CONTENT_ID]))
            {
            case eMsgID::EXAMPLE1:
                RoverCanLib::Helpers::canMsgToStruct<bool, UnionDefinition::BoolUnion>(msg_, &this->data.example1, logger_);
                break;

            case eMsgID::EXAMPLE2:
                RoverCanLib::Helpers::canMsgToStruct<float, UnionDefinition::FloatUnion>(msg_, &this->data.example2, logger_);
                break;

            case eMsgID::EXAMPLE3:
                RoverCanLib::Helpers::canMsgToStruct<uint16_t, UnionDefinition::Uint16_tUnion>(msg_, &this->data.example3, logger_);
                break;

            default:
                RCLCPP_WARN(logger_, "Unknown \"Message Specific Id\"");
                return Constant::eInternalErrorCode::ERROR;
            }

            return Constant::eInternalErrorCode::OK;
        }

        Constant::eInternalErrorCode getMsg(IN uint8_t msgId_, OUT can_frame *msg_, rclcpp::Logger logger_)
        {
            msg_->data[(uint8_t)Constant::eDataIndex::MSG_ID] = (uint8_t)Constant::eMsgId::ERROR_STATE;
            msg_->data[(uint8_t)Constant::eDataIndex::MSG_CONTENT_ID] = msgId_;

            switch ((RoverCanLib::Msgs::ExampleMsg::eMsgID)msgId_)
            {
            case eMsgID::EXAMPLE1:
                Helpers::structToCanMsg<bool, UnionDefinition::BoolUnion>(&data.example1, msg_);
                break;

            case eMsgID::EXAMPLE2:
                Helpers::structToCanMsg<float, UnionDefinition::FloatUnion>(&data.example2, msg_);
                break;

            case eMsgID::EXAMPLE3:
                Helpers::structToCanMsg<uint16_t, UnionDefinition::Uint16_tUnion>(&data.example3, msg_);
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

#endif // __EXAMPLE_MSG_HPP__
