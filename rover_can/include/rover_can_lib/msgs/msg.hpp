#ifndef __MSG_HPP__
#define __MSG_HPP__

#if defined(ESP32)
#include "driver/twai.h"
#include "helpers/helpers.hpp"
#endif // defined(ESP32)

#include "rover_can_lib/constant.hpp"

namespace RoverCanLib::Msgs
{
    class Msg
    {
    public:
        Msg(){}
        virtual ~Msg(){}

        virtual Constant::eInternalErrorCode parseMsg(const twai_message_t *msg_) = 0;
        virtual Constant::eInternalErrorCode getMsg(IN uint8_t msgId_, OUT twai_message_t* msg_) = 0;

        virtual uint8_t getMsgIDNb(void) = 0;
    };
}

#endif // __MSG_HPP__
