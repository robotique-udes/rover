#ifndef __JOY_CONTROLLER_HPP__
#define __JOY_CONTROLLER_HPP__

#include "keybinding.hpp"

#include "rovus_lib/timer.hpp"
#include "rovus_lib/macros.h"

#include <stdint.h>
#include <vector>
#include <map>
#include <initializer_list>

static constexpr float DEADZONE = 0.01F;
static constexpr float NEUTRAL = 0.0F;

class JoyManager
{
  public:
    ~JoyManager() = default;
    JoyManager():
        _joyInputArray{},
        _prevButtonStates{},
        _risingEdgeStates{}
    {
    }

    void updateJoyArray(std::array<float, TO_UNDERLYING(eJoyInput::eLAST)> joyInputArray_)
    {
        _joyInputArray = joyInputArray_;

        for (size_t i = 0; i < TO_UNDERLYING(eJoyInput::eLAST); ++i)
        {
            eJoyInput button = static_cast<eJoyInput>(i);
            bool isPressedNow = isPressed(button);
            _risingEdgeStates[i] = (isPressedNow && !_prevButtonStates[i]);
            _prevButtonStates[i] = isPressedNow;
        }
    }
    bool isPressed(eJoyInput joyInput_)
    {
        return !IN_ERROR(_joyInputArray[TO_UNDERLYING(joyInput_)], DEADZONE, NEUTRAL);
    }
    bool isTriggered(eJoyInput joyInput_)
    {
        return _risingEdgeStates[TO_UNDERLYING(joyInput_)];
    }

    std::array<float, TO_UNDERLYING(eJoyInput::eLAST)> getJoyArray(void)
    {
        return _joyInputArray;
    }

  protected:
    std::array<float, TO_UNDERLYING(eJoyInput::eLAST)> _joyInputArray;
    std::array<bool, TO_UNDERLYING(eJoyInput::eLAST)> _prevButtonStates;
    std::array<bool, TO_UNDERLYING(eJoyInput::eLAST)> _risingEdgeStates;
};

#endif
