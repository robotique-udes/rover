#ifndef __CONSTANT_HPP__
#define __CONSTANT_HPP__

#include <cstdint>
#include "rover_lib2/helpers/compile_time_array.hpp"

namespace RoverCan2::Constant
{
    constexpr size_t CAN_MAX_DATA_LENGTH = 8UL;
    constexpr float MASTER_HEARTBEAT_RATE_HZ = 10.0F;

    enum class eDeviceId : uint16_t
    {
        // clang-format off
    _RESERVED_MASTER = 0x020,
        MASTER_COMPUTER_UNIT = 0x020,
        BATTERY = 0x021,
        PDB_CONTROLLER = 0x022,
        DDB_CONTROLLER = 0x023,

    _RESERVED_PROPULSION = 0x100,
        FRONTLEFT_MOTOR = 0x101,
        FRONTRIGHT_MOTOR = 0x102,
        REARLEFT_MOTOR = 0x105,
        REARRIGHT_MOTOR = 0x106,

    _RESERVED_ARM = 0x200,
        JL_CONTROLLER = 0x201,
        JR_CONTROLLER = 0x202,
        J1_CONTROLLER = 0x203,
        J2_CONTROLLER = 0x204,
        GRIPPER_TILT_CONTROLLER = 0x205,
        GRIPPER_ROT_CONTROLLER = 0x206,
        GRIPPER_CLOSE_CONTROLLER = 0x207,
        GRIPPER_LASER = 0x208,
        GRIPPER_DISTANCE = 0x209,

    _RESERVED_GREEN_AUXILIARY = 0x300,
        GPS = 0x301,
        GNSS = 0x302,
        COMPASS = 0x303,
        LIGHTS_MAIN = 0x304,
        LIGHTS_IR = 0x305,

    _FREE_AUXILIARY = 0x400,
        CAMERA_ROVER_FPV = 0x401,
        CAMERA_ROVER_ANTENNA = 0x402,
        CAMERA_ROVER_FRONT = 0x403,
        CAMERA_ROVER_SCIENCE = 0x404,
        CAMERA_ARM_CENTER = 0x405,
        CAMERA_ARM_SIDE = 0x406,
        SPEAKERS = 0x407,

    _RESERVED_INTERNAL = 0x7F0,
        INVALID,
        NOT_SET = INVALID,

        TEST_DEVICE = 0x7FF

        // clang-format on
    };

    enum class eMsgId : uint8_t
    {
        // clang-format off
    _RESERVED = 0x00,
        INVALID = 0x00,

        TEST_MSG = 0x01,
        TEST_MSG_2 = 0x02,

    _FREE = 0x10,
        ERROR_STATE,
        HEARTBEAT,
        GPS,
        PROPULSION_MOTOR_CMD,
        PROPULSION_MOTOR_STATUS,
        CAM_CONTROL,
        CAM_CONTROL_A2,
        LIGHT_CONTROL,
        SCIENCE,
        COMPASS,
        ARM_CMD,
        ARM_STATUS,
        CAM_PAN,
        // clang-format on
    };

    /**
     * @brief Array holding all valid and implemented msgs used on the network.
     *
     */
    constexpr CompileTimeArray<eMsgId, 3UL> SUPPORTED_MSGS = {eMsgId::TEST_MSG, eMsgId::TEST_MSG_2, eMsgId::ERROR_STATE};

    /**
     * @brief
     * @attention [WARNING] Internal use only
     *
     */
    enum class eDataIndex : uint8_t
    {
        MSG_ID = 0x00,
        MSG_CONTENT_ID = 0x01,
        START_OF_DATA = 0x02
    };

}  // namespace RoverCan2::Constant

#endif  // __CONSTANT_HPP__
