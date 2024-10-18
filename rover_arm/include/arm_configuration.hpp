#ifndef _ARM_CONFIGURATION__
#define _ARM_CONFIGURATION__

namespace ARM_CONFIGURATION
{
    // Template for new joints
    // namespace <JOINT_NAME>
    // {
    //     constexpr float MAX_VELOCITY = <VALUE>
    //     namespace LEN
    //     {
    //         constexpr float x = <VALUE>
    //         constexpr float y = <VALUE>
    //         constexpr float z = <VALUE>
    //     }
    // }

    namespace JL
    {
        constexpr float MAX_VELOCITY = 0.05f;  // mm/s
        namespace LEN
        {
            constexpr float x = 0.0f;
            constexpr float y = 0.0f;
            constexpr float z = 0.0f;
        }  // namespace LEN
    }      // namespace JL

    namespace J0
    {
        constexpr float MAX_VELOCITY = 1.0f;  // rad/s
        namespace LEN
        {
            constexpr float x = 0.0f;
            constexpr float y = 0.0f;
            constexpr float z = 0.0f;
        }  // namespace LEN
    }      // namespace J0

    namespace J1
    {
        constexpr float MAX_VELOCITY = 1.0f;  // rad/s
        namespace LEN
        {
            constexpr float x = 0.0f;
            constexpr float y = 0.0f;
            constexpr float z = 0.0f;
        }  // namespace LEN
    }      // namespace J1

    namespace J2
    {
        constexpr float MAX_VELOCITY = 1.0f;  // rad/s
        namespace LEN
        {
            constexpr float x = 0.65f;
            constexpr float y = 0.0f;
            constexpr float z = 0.0f;
        }  // namespace LEN
    }      // namespace J2

    namespace GRIPPER_TILT
    {
        constexpr float MAX_VELOCITY = 1.0f;  // rad/s
        namespace LEN
        {
            constexpr float x = 0.62f;
            constexpr float y = 0.0f;
            constexpr float z = 0.0f;
        }  // namespace LEN
    }      // namespace GRIPPER_TILT

    namespace GRIPPER_ROT
    {
        constexpr float MAX_VELOCITY = 1.0f;  // rad/s
        namespace LEN
        {
            constexpr float x = 0.217f;
            constexpr float y = 0.0f;
            constexpr float z = 0.0f;
        }  // namespace LEN
    }      // namespace GRIPPER_ROT

    // GRIPPER_CLOSE not defined because no speed control available in current arm configuration
}  // namespace ARM_CONFIGURATION

// Length for MotionGenesis kinematics, defined for quicker back and forth,
// see doc or motion genesis code for bases reference
constexpr float J0x = ARM_CONFIGURATION::J0::LEN::x;
constexpr float J0y = ARM_CONFIGURATION::J0::LEN::y;
constexpr float J0z = ARM_CONFIGURATION::J0::LEN::z;

constexpr float J1x = ARM_CONFIGURATION::J1::LEN::x;
constexpr float J1y = ARM_CONFIGURATION::J1::LEN::y;
constexpr float J1z = ARM_CONFIGURATION::J1::LEN::z;

constexpr float J2x = ARM_CONFIGURATION::J2::LEN::x;
constexpr float J2y = ARM_CONFIGURATION::J2::LEN::y;
constexpr float J2z = ARM_CONFIGURATION::J2::LEN::z;

constexpr float J3x = ARM_CONFIGURATION::GRIPPER_TILT::LEN::x;
constexpr float J3y = ARM_CONFIGURATION::GRIPPER_TILT::LEN::y;
constexpr float J3z = ARM_CONFIGURATION::GRIPPER_TILT::LEN::z;

constexpr float J4x = ARM_CONFIGURATION::GRIPPER_ROT::LEN::x;
constexpr float J4y = ARM_CONFIGURATION::GRIPPER_ROT::LEN::y;
constexpr float J4z = ARM_CONFIGURATION::GRIPPER_ROT::LEN::z;

#endif  // __ARM_CONFIGURATION__
