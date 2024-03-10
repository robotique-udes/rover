#ifndef __PROPULSION_MOTOR_MSG_HPP__
#define __PROPULSION_MOTOR_MSG_HPP__

#include <cstdint>

namespace RoverCanLib::Msg::PropulsionMotor
{
    enum eMsgID : uint8_t
    {
        CLOSE_LOOP = 0x00,
        ENABLE = 0x01,
        TARGET_SPEED = 0x02,
        CURRENT_SPEED = 0x03,
        KP = 0x04,
        KI = 0x05,
        KD = 0x06
    };

    struct sMsgData
    {
        bool closeLoop = false;
        bool enable = false;
        float targetSpeed = 0.0f;
        float currentSpeed = 0.0f;
        float kp = 0.0f;
        float ki = 0.0f;
        float kd = 0.0f;
    };
};

#endif // __PROPULSION_MOTOR_MSG_HPP__
