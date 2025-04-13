#ifndef __KEYBINDING__HPP__
#define __KEYBINDING__HPP__

#include "arm_configuration.hpp"
namespace KEYBINDINGS
{
    namespace EMILE
    {
        constexpr uint8_t DEADMAN_SWITCH = rover_msgs::msg::Joy::L1;

        namespace JOINT
        {
            constexpr uint8_t JOINT_SELECT_INC = rover_msgs::msg::Joy::CROSS_UP;
            constexpr uint8_t JOINT_SELECT_DEC = rover_msgs::msg::Joy::CROSS_DOWN;

            constexpr uint8_t JL_RIGHT = rover_msgs::msg::Joy::CROSS_RIGHT;
            constexpr uint8_t JL_LEFT = rover_msgs::msg::Joy::CROSS_LEFT;

            constexpr uint8_t J1 = rover_msgs::msg::Joy::JOYSTICK_LEFT_FRONT;

            constexpr uint8_t J2 = rover_msgs::msg::Joy::JOYSTICK_RIGHT_FRONT;
        }  // namespace JOINT

        namespace CARTESIAN
        {
            constexpr uint8_t TOGGLE_CARTESIAN = rover_msgs::msg::Joy::Y;

            constexpr uint8_t X_AXIS_RIGHT = rover_msgs::msg::Joy::CROSS_RIGHT;
            constexpr uint8_t X_AXIS_LEFT = rover_msgs::msg::Joy::CROSS_LEFT;

            constexpr uint8_t Y_AXIS = rover_msgs::msg::Joy::JOYSTICK_RIGHT_FRONT;

            constexpr uint8_t Z_AXIS = rover_msgs::msg::Joy::JOYSTICK_LEFT_FRONT;

            constexpr uint8_t RECORD = rover_msgs::msg::Joy::A;
            constexpr uint8_t CLEAR_POINTS = rover_msgs::msg::Joy::X;
            constexpr uint8_t CREATE_PLAN = rover_msgs::msg::Joy::B;

        }  // namespace CARTESIAN

        namespace GRIPPER
        {
            constexpr uint8_t ACTIVATE_GRIPPER = rover_msgs::msg::Joy::R1;

            constexpr uint8_t ROT_FWD = rover_msgs::msg::Joy::B;
            constexpr uint8_t ROT_REV = rover_msgs::msg::Joy::X;

            constexpr uint8_t TILT_FWD = rover_msgs::msg::Joy::Y;
            constexpr uint8_t TILT_REV = rover_msgs::msg::Joy::A;

            constexpr uint8_t CLOSE = rover_msgs::msg::Joy::CROSS_DOWN;
        }  // namespace GRIPPER

    }  // namespace EMILE
}  // namespace KEYBINDINGS

#endif
