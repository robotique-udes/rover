#ifndef __CONSTANT_HPP__
#define __CONSTANT_HPP__

#include <cstdint>
#include "rover_lib2/helpers/compile_time_array.hpp"
#include "rover_lib2/LED/blink_pattern.hpp"

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
        
        POWER_CMD,
        POWER_STATUS,
        
        PWM_CMD,
        PWM_STATUS,
        PWM_INFO,

        PROP_SPEED_CMD,
        PROP_SPEED_STATUS,
        
        ARM_SPEED_CMD,
        ARM_POSITION_STATUS,
        ARM_JOINT_CONFIG,
        
        FIX_POSITION,
        FIX_HEADING,
        FIX_INFO,
        
        CAM_POSITION_CMD,
        CAM_POSITION_STATUS,

        DDB_CMD,
        DDB_STATUS,
        // clang-format on
    };

    /**
     * @brief Array holding all valid and implemented msgs used on the network.
     *
     */
    constexpr CompileTimeArray<eMsgId, 11UL> SUPPORTED_MSGS = {eMsgId::TEST_MSG,
                                                               eMsgId::TEST_MSG_2,
                                                               eMsgId::ERROR_STATE,
                                                               eMsgId::HEARTBEAT,
                                                               eMsgId::POWER_CMD,
                                                               eMsgId::POWER_STATUS,
                                                               eMsgId::PWM_CMD,
                                                               eMsgId::PWM_STATUS,
                                                               eMsgId::PWM_INFO,
                                                               eMsgId::DDB_CMD,
                                                               eMsgId::DDB_STATUS};

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

    namespace LedPatterns
    {
        constexpr LED::BlinkingPattern DRIVER_NOT_STARTED = LED::BlinkPatterns::OFF;
        constexpr LED::BlinkingPattern<4UL> RUNNING_OK = {LED::BlinkPatternStep(50UL, 100U),
                                                          LED::BlinkPatternStep(50UL, 25U),
                                                          LED::BlinkPatternStep(50UL, 100U),
                                                          LED::BlinkPatternStep(850UL, 25U)};
        constexpr LED::BlinkingPattern<9UL> DRIVER_INTERNAL_ERROR = {LED::BlinkPatternStep(200UL, 100U),
                                                                     LED::BlinkPatternStep(200UL, 0U),
                                                                     LED::BlinkPatternStep(200UL, 100U),
                                                                     LED::BlinkPatternStep(200UL, 0U),
                                                                     LED::BlinkPatternStep(200UL, 100U),
                                                                     LED::BlinkPatternStep(200UL, 0U),
                                                                     LED::BlinkPatternStep(200UL, 100U),
                                                                     LED::BlinkPatternStep(200UL, 0U),
                                                                     LED::BlinkPatternStep(2'000UL, 100U)};
        constexpr LED::BlinkingPattern<8UL> TX_QUEUE_FULL = {LED::BlinkPatternStep(50UL, 10U),
                                                             LED::BlinkPatternStep(50UL, 0U),
                                                             LED::BlinkPatternStep(50UL, 10U),
                                                             LED::BlinkPatternStep(50UL, 0U),
                                                             LED::BlinkPatternStep(50UL, 100U),
                                                             LED::BlinkPatternStep(50UL, 0U),
                                                             LED::BlinkPatternStep(50UL, 100U),
                                                             LED::BlinkPatternStep(50UL, 0U)};
        constexpr LED::BlinkingPattern WATCHDOG_TRIGGER = TX_QUEUE_FULL;
    }  // namespace LedPatterns

}  // namespace RoverCan2::Constant

#endif  // __CONSTANT_HPP__
