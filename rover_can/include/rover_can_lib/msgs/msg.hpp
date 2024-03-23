#ifndef __MSG_HPP__
#define __MSG_HPP__

#if defined(ESP32)
#include "driver/twai.h"
#include "helpers/helpers.hpp"
#elif defined(__linux__) // defined(ESP32)
#include <linux/can.h>
#include "rclcpp/rclcpp.hpp"
#include "rovus_lib/macros.h"
#endif // defined(ESP32)

#include "rover_can_lib/constant.hpp"

namespace RoverCanLib::Msgs
{
    class Msg
    {
    public:
        Msg(){}
        virtual ~Msg(){}

#if defined(ESP32)
        virtual Constant::eInternalErrorCode parseMsg(const twai_message_t *msg_) = 0;
        virtual Constant::eInternalErrorCode getMsg(IN uint8_t msgId_, OUT twai_message_t* msg_) = 0;
#elif defined (__linux__) // defined(ESP32)
        virtual Constant::eInternalErrorCode parseMsg(const can_frame *msg_, rclcpp::Logger logger_) = 0;
        virtual Constant::eInternalErrorCode getMsg(IN uint8_t msgId_, OUT can_frame* msg_, rclcpp::Logger logger_) = 0;
#endif // defined(ESP32)

        virtual uint8_t getMsgIDNb(void) = 0;
    };
}

#endif // __MSG_HPP__
