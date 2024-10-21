#ifndef __GPS_HPP__
#define __GPS_HPP__

#include "rover_can_lib/msgs/msg.hpp"
#include <cstdint>

#if defined(ESP32)
#include "driver/twai.h"
#elif defined(__linux__)  // defined(ESP32)
#include <linux/can.h>
#endif  // defined(ESP32)

#include "rover_can_lib/helpers.hpp"

namespace RoverCanLib::Msgs
{
    class GPS : public Msg
    {
      public:
        enum class eMsgID : uint8_t
        {
            NOT_USED = 0x00,
            LATITUDE = 0x01,
            LONGITUDE = 0x02,
            FIX = 0x03,
            eLAST
        };

        struct sMsgData
        {
            float latitude;
            float longitude;
            uint8_t fix;
        };

        GPS()
        {
            data.latitude = 0.0f;
            data.longitude = 0.0f;
            data.fix = 0u;
        }
        ~GPS() {}

#if defined(ESP32)
        Constant::eInternalErrorCode parseMsg(const twai_message_t* msg_)
        {
            if (msg_->data[(uint8_t)Constant::eDataIndex::MSG_ID] != (uint8_t)Constant::eMsgId::GPS)
            {
                LOG(ERROR, "Mismatch in message types, maybe the lib version isn't the same between all nodes... Dropping msg");
                return Constant::eInternalErrorCode::WARNING;
            }

            switch ((Msgs::GPS::eMsgID)(msg_->data[(uint8_t)Constant::eDataIndex::MSG_CONTENT_ID]))
            {
                case eMsgID::LATITUDE:
                    RoverCanLib::Helpers::canMsgToStruct<float, UnionDefinition::FloatUnion>(msg_, &this->data.latitude);
                    break;

                case eMsgID::LONGITUDE:
                    RoverCanLib::Helpers::canMsgToStruct<float, UnionDefinition::FloatUnion>(msg_, &this->data.longitude);
                    break;

                case eMsgID::FIX:
                    RoverCanLib::Helpers::canMsgToStruct<uint8_t, UnionDefinition::Uint8_tUnion>(msg_, &this->data.fix);
                    break;

                default: LOG(WARN, "Unknown \"Message Specific Id\""); return Constant::eInternalErrorCode::ERROR;
            }

            return Constant::eInternalErrorCode::OK;
        }

        Constant::eInternalErrorCode getMsg(IN uint8_t msgId_, OUT twai_message_t* msg_)
        {
            msg_->data[(uint8_t)Constant::eDataIndex::MSG_ID] = (uint8_t)Constant::eMsgId::GPS;
            msg_->data[(uint8_t)Constant::eDataIndex::MSG_CONTENT_ID] = msgId_;

            switch ((RoverCanLib::Msgs::GPS::eMsgID)msgId_)
            {
                case eMsgID::LATITUDE: Helpers::structToCanMsg<float, UnionDefinition::FloatUnion>(&data.latitude, msg_); break;

                case eMsgID::LONGITUDE: Helpers::structToCanMsg<float, UnionDefinition::FloatUnion>(&data.longitude, msg_); break;

                case eMsgID::FIX: Helpers::structToCanMsg<uint8_t, UnionDefinition::Uint8_tUnion>(&data.fix, msg_); break;

                default:
                    LOG(ERROR, "Shouldn't ever fall here, implementation error");
                    *msg_ = RoverCanLib::Helpers::getErrorIdMsg();
                    return Constant::eInternalErrorCode::ERROR;
            }

            return Constant::eInternalErrorCode::OK;
        }
#elif defined(__linux__)  // defined(ESP32)
        Constant::eInternalErrorCode parseMsg(const can_frame* msg_, rclcpp::Logger logger_)
        {
            if (msg_->data[(uint8_t)Constant::eDataIndex::MSG_ID] != (uint8_t)Constant::eMsgId::GPS)
            {
                RCLCPP_ERROR(logger_,
                             "Mismatch in message types, maybe the lib version isn't the same between all nodes... Dropping msg");
                return Constant::eInternalErrorCode::WARNING;
            }

            switch ((Msgs::GPS::eMsgID)(msg_->data[(uint8_t)Constant::eDataIndex::MSG_CONTENT_ID]))
            {
                case eMsgID::LATITUDE:
                    RoverCanLib::Helpers::canMsgToStruct<float, UnionDefinition::FloatUnion>(msg_, &this->data.latitude, logger_);
                    break;

                case eMsgID::LONGITUDE:
                    RoverCanLib::Helpers::canMsgToStruct<float, UnionDefinition::FloatUnion>(msg_,
                                                                                             &this->data.longitude,
                                                                                             logger_);
                    break;

                case eMsgID::FIX:
                    RoverCanLib::Helpers::canMsgToStruct<uint8_t, UnionDefinition::Uint8_tUnion>(msg_, &this->data.fix, logger_);
                    break;

                default: RCLCPP_WARN(logger_, "Unknown \"Message Specific Id\""); return Constant::eInternalErrorCode::ERROR;
            }

            return Constant::eInternalErrorCode::OK;
        }

        Constant::eInternalErrorCode getMsg(IN uint8_t msgId_, OUT can_frame* msg_, rclcpp::Logger logger_)
        {
            msg_->data[(uint8_t)Constant::eDataIndex::MSG_ID] = (uint8_t)Constant::eMsgId::GPS;
            msg_->data[(uint8_t)Constant::eDataIndex::MSG_CONTENT_ID] = msgId_;

            switch ((RoverCanLib::Msgs::GPS::eMsgID)msgId_)
            {
                case eMsgID::LATITUDE: Helpers::structToCanMsg<float, UnionDefinition::FloatUnion>(&data.latitude, msg_); break;

                case eMsgID::LONGITUDE: Helpers::structToCanMsg<float, UnionDefinition::FloatUnion>(&data.longitude, msg_); break;

                case eMsgID::FIX: Helpers::structToCanMsg<uint8_t, UnionDefinition::Uint8_tUnion>(&data.fix, msg_); break;

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

            static_assert((size_t)eMsgID::eLAST < UINT8_MAX);  // Make sure to not overflow counter
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
#endif                    // defined(ESP32)

        uint8_t getMsgIDNb(void)
        {
            return (uint8_t)eMsgID::eLAST;
        }

        sMsgData data;
    };
}  // namespace RoverCanLib::Msgs

#endif  // __GPS_HPP__
