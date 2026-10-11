#ifndef __KEYBINDING_ARM__HPP__
#define __KEYBINDING_ARM__HPP__

#include <rover_msgs/msg/joy.hpp>
#include <rover_lib2/helpers/constants.hpp>

namespace KEYBINDINGS
{
    constexpr Constants::Keybinds::eJoyInput DEADMAN_SWITCH = Constants::Keybinds::eJoyInput::L1;

    namespace JOINT
    {
        constexpr Constants::Keybinds::eJoyInput JL_RIGHT = Constants::Keybinds::eJoyInput::B;
        constexpr Constants::Keybinds::eJoyInput JL_LEFT = Constants::Keybinds::eJoyInput::X;

        constexpr Constants::Keybinds::eJoyInput J0
            = Constants::Keybinds::eJoyInput::eLAST;  // Is set to eLAST since rotating joint is not implemented

        constexpr Constants::Keybinds::eJoyInput J1 = Constants::Keybinds::eJoyInput::JOYSTICK_LEFT_FRONT;

        constexpr Constants::Keybinds::eJoyInput J2 = Constants::Keybinds::eJoyInput::JOYSTICK_RIGHT_FRONT;

        constexpr Constants::Keybinds::eJoyInput WRIST_UP = Constants::Keybinds::eJoyInput::CROSS_UP;
        constexpr Constants::Keybinds::eJoyInput WRIST_DOWN = Constants::Keybinds::eJoyInput::CROSS_DOWN;
        constexpr Constants::Keybinds::eJoyInput WRIST_ROT_LEFT = Constants::Keybinds::eJoyInput::CROSS_LEFT;
        constexpr Constants::Keybinds::eJoyInput WRIST_ROT_RIGHT = Constants::Keybinds::eJoyInput::CROSS_RIGHT;
        constexpr Constants::Keybinds::eJoyInput GRIPPER_CLOSE = Constants::Keybinds::eJoyInput::R2;
        constexpr Constants::Keybinds::eJoyInput GRIPPER_OPEN = Constants::Keybinds::eJoyInput::L2;

    }  // namespace JOINT

    namespace CARTESIAN
    {
        constexpr Constants::Keybinds::eJoyInput TOGGLE_CARTESIAN = Constants::Keybinds::eJoyInput::Y;

        constexpr Constants::Keybinds::eJoyInput X_AXIS = Constants::Keybinds::eJoyInput::JOYSTICK_LEFT_SIDE;

        constexpr Constants::Keybinds::eJoyInput Y_AXIS = Constants::Keybinds::eJoyInput::JOYSTICK_LEFT_FRONT;

        constexpr Constants::Keybinds::eJoyInput Z_AXIS = Constants::Keybinds::eJoyInput::JOYSTICK_RIGHT_FRONT;

        constexpr Constants::Keybinds::eJoyInput WRIST_ROT_LEFT = Constants::Keybinds::eJoyInput::CROSS_LEFT;
        constexpr Constants::Keybinds::eJoyInput WRIST_ROT_RIGHT = Constants::Keybinds::eJoyInput::CROSS_RIGHT;
        constexpr Constants::Keybinds::eJoyInput GRIPPER_CLOSE = Constants::Keybinds::eJoyInput::L2;
        constexpr Constants::Keybinds::eJoyInput GRIPPER_OPEN = Constants::Keybinds::eJoyInput::R2;

        constexpr Constants::Keybinds::eJoyInput ALPHA_POSITIVE = Constants::Keybinds::eJoyInput::CROSS_UP;
        constexpr Constants::Keybinds::eJoyInput ALPHA_NEGATIVE = Constants::Keybinds::eJoyInput::CROSS_DOWN;

        constexpr Constants::Keybinds::eJoyInput BOMBO_SPEED = Constants::Keybinds::eJoyInput::R1;

        // constexpr Constants::Keybinds::eJoyInput RECORD = Constants::Keybinds::eJoyInput::A;
        // constexpr Constants::Keybinds::eJoyInput CLEAR_POINTS = Constants::Keybinds::eJoyInput::X;
        // constexpr Constants::Keybinds::eJoyInput CREATE_PLAN = Constants::Keybinds::eJoyInput::B;

    }  // namespace CARTESIAN

}  // namespace KEYBINDINGS

#endif
