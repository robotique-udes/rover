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
    ~JoyController() = default;
    JoyController() = default;

    bool isPressed(float buttonValue_)
    {
        return !IN_ERROR(buttonValue_, 0.01F, 0.0F);
    }

    bool isSelected(float buttonValue_, uint8_t buttonId)
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
    std::map<uint8_t, bool> _buttonStates = {{rover_msgs::msg::Joy::CROSS_UP, false},
                                             {rover_msgs::msg::Joy::CROSS_DOWN, false},
                                             {rover_msgs::msg::Joy::CROSS_RIGHT, false},
                                             {rover_msgs::msg::Joy::CROSS_LEFT, false},
                                             {rover_msgs::msg::Joy::A, false},
                                             {rover_msgs::msg::Joy::B, false},
                                             {rover_msgs::msg::Joy::X, false},
                                             {rover_msgs::msg::Joy::Y, false},
                                             {rover_msgs::msg::Joy::L1, false},
                                             {rover_msgs::msg::Joy::L2, false}};
};

#endif
