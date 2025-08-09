#ifndef __KEYBINDING__HPP__
#define __KEYBINDING__HPP__

#include <rover_msgs/msg/joy.hpp>

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

enum class eBooleanJoyInputs
{
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

    constexpr eJoyInput DEADMAN_SWITCH = eJoyInput::L1;

    namespace JOINT
    {
        constexpr eJoyInput JL_RIGHT = eJoyInput::B;
        constexpr eJoyInput JL_LEFT = eJoyInput::X;

        constexpr eJoyInput J0 = eJoyInput::eLAST;  // Is set to eLAST since rotating joint is not implemented

        constexpr eJoyInput J1 = eJoyInput::JOYSTICK_LEFT_FRONT;

        constexpr eJoyInput J2 = eJoyInput::JOYSTICK_RIGHT_FRONT;

        constexpr eJoyInput WRIST_UP = eJoyInput::CROSS_UP;
        constexpr eJoyInput WRIST_DOWN = eJoyInput::CROSS_DOWN;
        constexpr eJoyInput WRIST_ROT_LEFT = eJoyInput::CROSS_LEFT;
        constexpr eJoyInput WRIST_ROT_RIGHT = eJoyInput::CROSS_RIGHT;
        constexpr eJoyInput GRIPPER_CLOSE = eJoyInput::R2;
        constexpr eJoyInput GRIPPER_OPEN = eJoyInput::L2;

    }  // namespace JOINT

    namespace CARTESIAN
    {
        constexpr eJoyInput TOGGLE_CARTESIAN = eJoyInput::Y;

        constexpr eJoyInput X_AXIS_RIGHT = eJoyInput::B;
        constexpr eJoyInput X_AXIS_LEFT = eJoyInput::X;

        constexpr eJoyInput Y_AXIS = eJoyInput::JOYSTICK_RIGHT_FRONT;

        constexpr eJoyInput Z_AXIS = eJoyInput::JOYSTICK_LEFT_FRONT;

        constexpr eJoyInput WRIST_UP = eJoyInput::CROSS_UP;
        constexpr eJoyInput WRIST_DOWN = eJoyInput::CROSS_DOWN;
        constexpr eJoyInput WRIST_ROT_LEFT = eJoyInput::CROSS_LEFT;
        constexpr eJoyInput WRIST_ROT_RIGHT = eJoyInput::CROSS_RIGHT;
        constexpr eJoyInput GRIPPER_CLOSE = eJoyInput::R2;
        constexpr eJoyInput GRIPPER_OPEN = eJoyInput::L2;

        constexpr eJoyInput ACTIVATE_ALPHA = eJoyInput::R1;
        constexpr eJoyInput ALPHA_POSITIVE = eJoyInput::R2;
        constexpr eJoyInput ALPHA_NEGATIVE = eJoyInput::L2;

        // constexpr eJoyInput RECORD = eJoyInput::A;
        // constexpr eJoyInput CLEAR_POINTS = eJoyInput::X;
        // constexpr eJoyInput CREATE_PLAN = eJoyInput::B;

    }  // namespace CARTESIAN

}  // namespace KEYBINDINGS

#endif
