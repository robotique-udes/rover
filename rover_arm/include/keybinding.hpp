#ifndef __KEYBINDING__
#define __KEYBINDING__

#include "rover_msgs/msg/arm_msg.hpp"

// Driver
#define PHIL

namespace KEYBINDING
{
#if defined (PHIL)
    constexpr uint8_t JOINT_SELECT_INC = rover_msgs::msg::Joy::CROSS_UP;
    constexpr uint8_t JOINT_SELECT_DEC = rover_msgs::msg::Joy::CROSS_DOWN;

    constexpr uint8_t JL_FWD = rover_msgs::msg::Joy::CROSS_LEFT;
    constexpr uint8_t JL_REV = rover_msgs::msg::Joy::CROSS_RIGHT;

    constexpr uint8_t J0_FWD = rover_msgs::msg::Joy::X;
    constexpr uint8_t J0_REV = rover_msgs::msg::Joy::B;

    constexpr uint8_t J1 = rover_msgs::msg::Joy::JOYSTICK_LEFT_FRONT;

    constexpr uint8_t J2 = rover_msgs::msg::Joy::JOYSTICK_RIGHT_FRONT;

    constexpr uint8_t GRIPPER_TILT_FWD = rover_msgs::msg::Joy::Y;
    constexpr uint8_t GRIPPER_TILT_REV = rover_msgs::msg::Joy::A;

    constexpr uint8_t GRIPPER_ROT_FWD = rover_msgs::msg::Joy::R2;
    constexpr uint8_t GRIPPER_ROT_REV = rover_msgs::msg::Joy::L2;

    constexpr uint8_t GRIPPER_CLOSE = rover_msgs::msg::Joy::CROSS_DOWN;
# endif // defined(PHIL)
}

#endif // __KEYBINDING__
