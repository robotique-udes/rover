#ifndef __SCIENCE_HPP__
#define __SCIENCE_HPP__

#include <cstdint>
#include "rover_can_lib/msgs/msg.hpp"

#if defined(ESP32)
#include "driver/twai.h"
#elif defined(__linux__) // defined(ESP32)
#include <linux/can.h>
#endif // defined(ESP32)

#include "rover_can_lib/helpers.hpp"

namespace RoverCanLib::Msgs
{
    class Science : public Msg
    {
    public:
        enum class eMsgID : uint8_t
        {
            NOT_USED = 0x00,
            CMD = 0x01,
            DRILL = 0x02,
            eLAST
        };

        struct sMsgData
        {
            int8_t cmd;
            int8_t drill;
        };

        Science() 
        {
            data.cmd = 0;
            data.drill = 0;
        }
        ~Science() {}

#if defined(ESP32)
        Constant::eInternalErrorCode parseMsg(const twai_message_t *msg_)
        {
            if (msg_->data[(uint8_t)Constant::eDataIndex::MSG_ID] != (uint8_t)Constant::eMsgId::SCIENCE)
            {
                LOG(ERROR, "Mismatch in message types, maybe the lib version isn't the same between all nodes... Dropping msg");
                return Constant::eInternalErrorCode::WARNING;
            }

            switch ((Msgs::Science::eMsgID)(msg_->data[(uint8_t)Constant::eDataIndex::MSG_CONTENT_ID]))
            {
            case eMsgID::CMD:
                RoverCanLib::Helpers::canMsgToStruct<int8_t, UnionDefinition::Int8_tUnion>(msg_, &this->data.cmd);
                break;

            case eMsgID::DRILL:
                RoverCanLib::Helpers::canMsgToStruct<int8_t, UnionDefinition::Int8_tUnion>(msg_, &this->data.drill);
                break;

            default:
                LOG(WARN, "Unknown \"Message Specific Id\"");
                return Constant::eInternalErrorCode::ERROR;
            }

            return Constant::eInternalErrorCode::OK;
        }

        Constant::eInternalErrorCode getMsg(IN uint8_t msgId_, OUT twai_message_t *msg_)
        {
            msg_->data[(uint8_t)Constant::eDataIndex::MSG_ID] = (uint8_t)Constant::eMsgId::SCIENCE;
            msg_->data[(uint8_t)Constant::eDataIndex::MSG_CONTENT_ID] = msgId_;

            switch ((RoverCanLib::Msgs::Science::eMsgID)msgId_)
            {
            case eMsgID::CMD:
                Helpers::structToCanMsg<int8_t, UnionDefinition::Int8_tUnion>(&data.cmd, msg_);
                break;

            case eMsgID::DRILL:
                Helpers::structToCanMsg<int8_t, UnionDefinition::Int8_tUnion>(&data.drill, msg_);
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
            if (msg_->data[(uint8_t)Constant::eDataIndex::MSG_ID] != (uint8_t)Constant::eMsgId::SCIENCE)
            {
                RCLCPP_ERROR(logger_, "Mismatch in message types, maybe the lib version isn't the same between all nodes... Dropping msg");
                return Constant::eInternalErrorCode::WARNING;
            }

            switch ((Msgs::Science::eMsgID)(msg_->data[(uint8_t)Constant::eDataIndex::MSG_CONTENT_ID]))
            {
            case eMsgID::CMD:
                RoverCanLib::Helpers::canMsgToStruct<int8_t, UnionDefinition::Int8_tUnion>(msg_, &this->data.cmd, logger_);
                break;

            case eMsgID::DRILL:
                RoverCanLib::Helpers::canMsgToStruct<int8_t, UnionDefinition::Int8_tUnion>(msg_, &this->data.drill, logger_);
                break;

            default:
                RCLCPP_WARN(logger_, "Unknown \"Message Specific Id\"");
                return Constant::eInternalErrorCode::ERROR;
            }

            return Constant::eInternalErrorCode::OK;
        }

        Constant::eInternalErrorCode getMsg(IN uint8_t msgId_, OUT can_frame *msg_, rclcpp::Logger logger_)
        {
            msg_->data[(uint8_t)Constant::eDataIndex::MSG_ID] = (uint8_t)Constant::eMsgId::SCIENCE;
            msg_->data[(uint8_t)Constant::eDataIndex::MSG_CONTENT_ID] = msgId_;

            switch ((RoverCanLib::Msgs::Science::eMsgID)msgId_)
            {
            case eMsgID::CMD:
                Helpers::structToCanMsg<int8_t, UnionDefinition::Int8_tUnion>(&data.cmd, msg_);
                break;

            case eMsgID::DRILL:
                Helpers::structToCanMsg<int8_t, UnionDefinition::Int8_tUnion>(&data.drill, msg_);
                break;

            default:
                RCLCPP_ERROR(logger_, "Shouldn't ever fall here, implementation error");
                *msg_ = RoverCanLib::Helpers::getErrorIdMsg();
                return Constant::eInternalErrorCode::ERROR;
            }

            return Constant::eInternalErrorCode::OK;
        }
        
        Constant::eInternalErrorCode sendMsg(RoverCanLib::Constant::eDeviceId deviceID_, int canSocket_, rclcpp::Logger logger_)
        {
            can_frame canFrame;
            canFrame.can_id = (canid_t)deviceID_;

            static_assert((size_t)eMsgID::eLAST < UINT8_MAX); // Make sure to not overflow counter
            for (uint8_t i = (uint8_t)eMsgID::NOT_USED + 1u; i < (uint8_t)eMsgID::eLAST; i++)
            {
                this->getMsg(i, &canFrame, logger_);
                if (write(canSocket_, &canFrame, sizeof(canFrame)) != sizeof(canFrame))
                {
                    RCLCPP_ERROR(logger_, "Error while sending error state msg");
                    return RoverCanLib::Constant::eInternalErrorCode::ERROR;
                }
            }

            return RoverCanLib::Constant::eInternalErrorCode::OK;
        }
#endif // defined(ESP32)

        uint8_t getMsgIDNb(void)
        {
            return (uint8_t)eMsgID::eLAST;
        }

        sMsgData data;
    };
}

#endif // __SCIENCE_HPP__
