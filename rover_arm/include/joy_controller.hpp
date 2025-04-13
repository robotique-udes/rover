#ifndef __JOY_CONTROLLER_HPP__
#define __JOY_CONTROLLER_HPP__

#include "keybinding.hpp"

#include "rovus_lib/timer.hpp"
#include "rovus_lib/macros.h"

#include <stdint.h>
#include <vector>
#include <map>
#include <initializer_list>

class JoyController
{
  public:
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

    ~JoyController() = default;
    JoyController() = default;

    bool isPressed(float buttonValue_)
    {
        return !IN_ERROR(buttonValue_, 0.01F, 0.0F);
    }

    bool isSelected(float buttonValue_, eJoyInput buttonId)
    {
        if (isPressed(buttonValue_) && !_buttonStates[buttonId])
        {
            _buttonStates[buttonId] = true;
            return true;
        }
        else if (!isPressed(buttonValue_) && _buttonStates[buttonId])
        {
            _buttonStates[buttonId] = false;
            return false;
        }
        return false;
    }

    uint8_t getNJoints(void)
    {
        return _nJoints;
    }

  protected:
    uint8_t _nJoints;
    std::vector<uint8_t> _joints;
    std::map<eJoyInput, bool> _buttonStates = {{eJoyInput::CROSS_UP, false},
                                               {eJoyInput::CROSS_DOWN, false},
                                               {eJoyInput::CROSS_RIGHT, false},
                                               {eJoyInput::CROSS_LEFT, false},
                                               {eJoyInput::A, false},
                                               {eJoyInput::B, false},
                                               {eJoyInput::X, false},
                                               {eJoyInput::Y, false},
                                               {eJoyInput::L1, false},
                                               {eJoyInput::L2, false}};
};

#endif
