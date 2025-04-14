#ifndef __KEYBINDING__HPP__
#define __KEYBINDING__HPP__

#include "arm_configuration.hpp"

enum class eJoyInput
{
    JOYSTICK_LEFT_FRONT = rover_msgs::msg::Joy::JOYSTICK_LEFT_FRONT,
    JOYSTICK_LEFT_SIDE = rover_msgs::msg::Joy::JOYSTICK_LEFT_SIDE,
    JOYSTICK_LEFT_PUSH = rover_msgs::msg::Joy::JOYSTICK_LEFT_PUSH,
    JOYSTICK_RIGHT_FRONT = rover_msgs::msg::Joy::JOYSTICK_RIGHT_FRONT,
    JOYSTICK_RIGHT_SIDE = rover_msgs::msg::Joy::JOYSTICK_RIGHT_SIDE,
    JOYSTICK_RIGHT_PUSH = rover_msgs::msg::Joy::JOYSTICK_RIGHT_PUSH,
    CROSS_UP = rover_msgs::msg::Joy::CROSS_UP,
    CROSS_DOWN = rover_msgs::msg::Joy::CROSS_DOWN,
    CROSS_LEFT = rover_msgs::msg::Joy::CROSS_LEFT,
    CROSS_RIGHT = rover_msgs::msg::Joy::CROSS_RIGHT,
    L1 = rover_msgs::msg::Joy::L1,
    L2 = rover_msgs::msg::Joy::L2,
    R1 = rover_msgs::msg::Joy::R1,
    R2 = rover_msgs::msg::Joy::R2,
    A = rover_msgs::msg::Joy::A,
    B = rover_msgs::msg::Joy::B,
    X = rover_msgs::msg::Joy::X,
    Y = rover_msgs::msg::Joy::Y,
    EXT0 = rover_msgs::msg::Joy::EXT0,
    EXT1 = rover_msgs::msg::Joy::EXT1,
    EXT2 = rover_msgs::msg::Joy::EXT2,
    eLAST
};

namespace KEYBINDINGS
{
    namespace EMILE
    {
        constexpr eJoyInput DEADMAN_SWITCH = eJoyInput::L1;

        namespace JOINT
        {
            constexpr eJoyInput JOINT_SELECT_INC = eJoyInput::CROSS_UP;
            constexpr eJoyInput JOINT_SELECT_DEC = eJoyInput::CROSS_DOWN;

            constexpr eJoyInput JL_RIGHT = eJoyInput::CROSS_RIGHT;
            constexpr eJoyInput JL_LEFT = eJoyInput::CROSS_LEFT;

            constexpr eJoyInput J0 = eJoyInput::eLAST; // Is set to eLAST since rotating joint is not implemented

            constexpr eJoyInput J1 = eJoyInput::JOYSTICK_LEFT_FRONT;

            constexpr eJoyInput J2 = eJoyInput::JOYSTICK_RIGHT_FRONT;
        }  // namespace JOINT

        namespace CARTESIAN
        {
            constexpr eJoyInput TOGGLE_CARTESIAN = eJoyInput::Y;

            constexpr eJoyInput X_AXIS_RIGHT = eJoyInput::CROSS_RIGHT;
            constexpr eJoyInput X_AXIS_LEFT = eJoyInput::CROSS_LEFT;

            constexpr eJoyInput Y_AXIS = eJoyInput::JOYSTICK_RIGHT_FRONT;

            constexpr eJoyInput Z_AXIS = eJoyInput::JOYSTICK_LEFT_FRONT;

            constexpr eJoyInput RECORD = eJoyInput::A;
            constexpr eJoyInput CLEAR_POINTS = eJoyInput::X;
            constexpr eJoyInput CREATE_PLAN = eJoyInput::B;

        }  // namespace CARTESIAN

        namespace GRIPPER
        {
            constexpr eJoyInput ACTIVATE_GRIPPER = eJoyInput::R1;

            constexpr eJoyInput ROT_FWD = eJoyInput::B;
            constexpr eJoyInput ROT_REV = eJoyInput::X;

            constexpr eJoyInput TILT_FWD = eJoyInput::Y;
            constexpr eJoyInput TILT_REV = eJoyInput::A;

            constexpr eJoyInput CLOSE = eJoyInput::CROSS_DOWN;
        }  // namespace GRIPPER

    }  // namespace EMILE
}  // namespace KEYBINDINGS

#endif
