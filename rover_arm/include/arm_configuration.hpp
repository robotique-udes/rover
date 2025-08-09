#ifndef _ARM_CONFIGURATION__HPP__
#define _ARM_CONFIGURATION__HPP__

#include <cstddef>
#include <rover_msgs/msg/arm_msg.hpp>

enum class eJointIndex : size_t
{
    JL = rover_msgs::msg::ArmMsg::JL,
    J0 = rover_msgs::msg::ArmMsg::J0,
    J1 = rover_msgs::msg::ArmMsg::J1,
    J2 = rover_msgs::msg::ArmMsg::J2,
    GRIPPER_TILT = rover_msgs::msg::ArmMsg::GRIPPER_TILT,
    GRIPPER_ROT = rover_msgs::msg::ArmMsg::GRIPPER_ROT,
    GRIPPER_CLOSE = rover_msgs::msg::ArmMsg::GRIPPER_CLOSE,
    eLAST
};

namespace ARM_CONFIGURATION
{
    // Template for new joints
    // namespace <JOINT_NAME>
    // {
    //     constexpr float MAX_VELOCITY = <VALUE>
    //     constexpr float JOG_VELOCITY = <VALUE>

    //     namespace LEN
    //     {
    //         constexpr float x = <VALUE>
    //         constexpr float y = <VALUE>
    //         constexpr float z = <VALUE>
    //     }
    // }

    namespace JL
    {
        constexpr eJointIndex ID = eJointIndex::JL;
        constexpr float MAX_VELOCITY = 0.035F;  // m/s
        constexpr float JOG_VELOCITY = 0.03F;   // m/s
        constexpr float CART_VELOCITY = 0.015F;  // m/s
        namespace LEN
        {
            constexpr float x = 0.0f;
            constexpr float y = 0.0f;
            constexpr float z = 0.0f;
        }  // namespace LEN
    }      // namespace JL

    namespace J0
    {
        constexpr eJointIndex ID = eJointIndex::J0;
        constexpr float MAX_VELOCITY = 0.05F;   // m/s
        constexpr float JOG_VELOCITY = 0.05F;   // m/s
        constexpr float CART_VELOCITY = 0.025F;  // m/s
        namespace LEN
        {
            constexpr float x = 0.0f;
            constexpr float y = 0.0f;
            constexpr float z = 0.0f;
        }  // namespace LEN
    }      // namespace J0

    namespace J1
    {
        constexpr eJointIndex ID = eJointIndex::J1;
        constexpr float MAX_VELOCITY = 0.2F;    // rad/s
        constexpr float JOG_VELOCITY = 0.15F;   // rad/s
        constexpr float CART_VELOCITY = 0.075F;  // rad/s

        namespace LEN
        {
            constexpr float x = 0.0F;
            constexpr float y = 0.0F;
            constexpr float z = 0.41F;
        }  // namespace LEN
    }      // namespace J1

    namespace J2
    {
        constexpr eJointIndex ID = eJointIndex::J2;
        constexpr float MAX_VELOCITY = 0.2F;    // rad/s
        constexpr float JOG_VELOCITY = 0.15F;   // rad/s
        constexpr float CART_VELOCITY = 0.75F;  // rad/s
        namespace LEN
        {
            constexpr float x = 0.0F;
            constexpr float y = 0.0F;
            constexpr float z = 0.415F;
        }  // namespace LEN
    }      // namespace J2

    namespace GRIPPER_TILT
    {
        constexpr eJointIndex ID = eJointIndex::GRIPPER_TILT;
        constexpr float MAX_VELOCITY = 0.8F;   // rad/s
        constexpr float JOG_VELOCITY = 0.2F;   // rad/s
        constexpr float CART_VELOCITY = 0.1F;  // rad/s

        namespace LEN
        {
            constexpr float x = 0.0F;
            constexpr float y = 0.0F;
            constexpr float z = 0.25F;
        }  // namespace LEN
    }      // namespace GRIPPER_TILT

    namespace GRIPPER_ROT
    {
        constexpr eJointIndex ID = eJointIndex::GRIPPER_ROT;
        constexpr float MAX_VELOCITY = 0.8;    // rad/s
        constexpr float JOG_VELOCITY = 0.8F;   // rad/s
        constexpr float CART_VELOCITY = 0.2F;  // rad/s

        namespace LEN
        {
            constexpr float x = 0.0F;
            constexpr float y = 0.0F;
            constexpr float z = 0.0F;
        }  // namespace LEN
    }      // namespace GRIPPER_ROT

    namespace GRIPPER_CLOSE
    {
        constexpr eJointIndex ID = eJointIndex::GRIPPER_CLOSE;
        constexpr float MAX_VELOCITY = 0.5F;   // rad/s
        constexpr float JOG_VELOCITY = 0.5F;   // rad/s
        constexpr float CART_VELOCITY = 0.25F;  // rad/s

        namespace LEN
        {
            constexpr float x = 0.0F;
            constexpr float y = 0.0F;
            constexpr float z = 0.0F;
        }  // namespace LEN
    }      // namespace GRIPPER_CLOSE

    // GRIPPER_CLOSE not defined because no speed control available in current arm configuration
}  // namespace ARM_CONFIGURATION

// Length for MotionGenesis kinematics, defined for quicker back and forth,
// see doc or motion genesis code for bases reference

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
