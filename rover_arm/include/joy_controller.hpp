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

  protected:
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
