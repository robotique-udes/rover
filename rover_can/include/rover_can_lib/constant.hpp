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
        PDB_CONTROLLER = 0x022,

        // PROPULSION
        FRONTLEFT_MOTOR = 0x101,
        FRONTRIGHT_MOTOR = 0x102,

        REARLEFT_MOTOR = 0x105,
        REARRIGHT_MOTOR = 0x106,

        // ARM
        JL_CONTROLLER = 0x201,
        J0_CONTROLLER = 0x202,
        J1_CONTROLLER = 0x203,
        J2_CONTROLLER = 0x204,
        GRIPPER_TILT_CONTROLLER = 0x205,
        GRIPPER_ROT_CONTROLLER = 0x206,
        GRIPPER_CLOSE_CONTROLLER = 0x207,

        // Science
        SCIENCE = 0x301,

        // Accessory
        GPS = 0x401,
        LIGHTS = 0x402,
        INFRARED_LIGHTS = 0x402,
        COMPASS = 0x404,
        CAMERA_A2 = 0x411,
        CAMERA_R1M_1 = 0x412,
        CAMERA_R1M_2 = 0x413,
        CAMERA_R1M_3 = 0x414,
        CAMERA_PAN = 0x415,
        SPEAKERS = 0x421,

        // Free Spaces
    };

    enum class eMsgId : uint8_t
    {
        NOT_USED = 0x00,
        ERROR_STATE = 0x01,
        HEARTBEAT = 0x02,
        GPS = 0x10,
        PROPULSION_MOTOR_CMD = 0x11,
        PROPULSION_MOTOR_STATUS = 0x12,
        CAM_CONTROL = 0x13,
        CAM_CONTROL_A2 = 0x14,
        LIGHT_CONTROL = 0x15,
        SCIENCE = 0x16,
        COMPASS = 0x17,
        ARM_CMD = 0x18,
        ARM_STATUS = 0x19,
        CAM_PAN = 0x20
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

}  // namespace RoverCanLib::Constant

#endif  // __CONSTANT_HPP__
