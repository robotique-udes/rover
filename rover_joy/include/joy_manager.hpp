#ifndef __JOY_CONTROLLER_HPP__
#define __JOY_CONTROLLER_HPP__

#include <rover_lib2/helpers/constants.hpp>
#include <rover_lib2/helpers/macros.hpp>
#include <stdint.h>

class JoyManager
{
    static constexpr float DEADZONE = 0.01F;
    static constexpr float NEUTRAL = 0.0F;

  public:
    void updateJoyArray(const std::array<float, TO_UNDERLYING(Constants::Keybinds::eJoyInput::eLAST)>& joyInputArray_)
    {
        _joyInputArray = joyInputArray_;

        for (size_t i = 0; i < TO_UNDERLYING(Constants::Keybinds::eJoyInput::eLAST); ++i)
        {
            Constants::Keybinds::eJoyInput button = static_cast<Constants::Keybinds::eJoyInput>(i);
            bool isPressedNow = isPressed(button);
            _risingEdgeStates[i] = (isPressedNow && !_prevButtonStates[i]);
            _prevButtonStates[i] = isPressedNow;
        }
    }
    bool isPressed(Constants::Keybinds::eJoyInput joyInput_)
    {
        return !IN_ERROR(_joyInputArray[TO_UNDERLYING(joyInput_)], DEADZONE, NEUTRAL);
    }
    bool isTriggered(Constants::Keybinds::eJoyInput joyInput_)
    {
        return _risingEdgeStates[TO_UNDERLYING(joyInput_)];
    }

    std::array<float, TO_UNDERLYING(Constants::Keybinds::eJoyInput::eLAST)> getJoyArray(void) const
    {
        return _joyInputArray;
    }

  private:
    std::array<float, TO_UNDERLYING(Constants::Keybinds::eJoyInput::eLAST)> _joyInputArray;
    std::array<bool, TO_UNDERLYING(Constants::Keybinds::eJoyInput::eLAST)> _prevButtonStates;
    std::array<bool, TO_UNDERLYING(Constants::Keybinds::eJoyInput::eLAST)> _risingEdgeStates;
};

#endif
