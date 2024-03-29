#ifndef __CONSTANT_HPP__
#define __CONSTANT_HPP__

#include <cstdint>

namespace RoverCanLib::Constant
{
    enum class eDeviceId : uint16_t
    {
        // RESERVED_FOR_MASTER = 0x000,
        MASTER_COMPUTER_UNIT = 0x020,
        BMS = 0x021,

        // PROPULSION
        FRONTLEFT_MOTOR = 0x101,
        FRONTRIGHT_MOTOR = 0x102,

        REARLEFT_MOTOR = 0x105,
        REARRIGHT_MOTOR = 0x106,

        // ARM
        J0_CONTROLLER = 0x210,
        J1_CONTROLLER = 0x211,
        J2_CONTROLLER = 0x212,
        J3_CONTROLLER = 0x213,
        J4_CONTROLLER = 0x214,
        J5_CONTROLLER = 0x215,
        J6_CONTROLLER = 0x216,

        // Science

        // Accessory
        GPS = 0x401,
        LIGHTS = 0x402,
        INFRARED_LIGHTS = 0x402,
        SPEAKERS = 0x410,

        // Free Space
    };

    enum class eMsgId : uint8_t
    {
        NOT_USED = 0x00,
        ERROR_STATE = 0x01,
        HEARTBEAT = 0x02,
        GPS = 0x10,
        PROPULSION_MOTOR_CMD = 0x11,
        PROPULSION_MOTOR_STATUS = 0x12
    };

    enum class eDataIndex : uint8_t
    {
        MSG_ID = 0x00,
        MSG_CONTENT_ID = 0x01,
        START_OF_DATA = 0x02
    };

    enum class eInternalErrorCode : uint8_t
    {
        OK,
        WARNING,
        ERROR
    };

}

#endif // __CONSTANT_HPP__
