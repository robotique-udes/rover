#ifndef ROVER_LIB2_SENSORS_PUSH_BUTTON_HPP
#define ROVER_LIB2_SENSORS_PUSH_BUTTON_HPP

#include "rover_lib2/IO/digital_input.hpp"

class PushButton
{
  public:
    enum class ePullMode
    {
        PULL_UP,
        PULL_DOWN
    };

    PushButton(gpio_num_t pin_,
               ePullMode pullMode_ = ePullMode::PULL_UP,
               gpio_pull_mode_t internalPullMode_ = gpio_pull_mode_t::GPIO_FLOATING):
        _io(pin_, gpio_mode_t::GPIO_MODE_INPUT, internalPullMode_),
        _pullMode(pullMode_)
    {
        ASSERT_COND_MSG(!(_pullMode == ePullMode::PULL_DOWN && internalPullMode_ == gpio_pull_mode_t::GPIO_PULLUP_ONLY)
                            && !(_pullMode == ePullMode::PULL_UP && internalPullMode_ == gpio_pull_mode_t::GPIO_PULLDOWN_ONLY),
                        "Invalid pull mode selected in relation to internal pull mode selected, expected undefined behavior");
    }

    bool isClicked(void)
    {
        IO::eIOState pinLevel = _io.read();
        if (_pullMode == ePullMode::PULL_UP)
        {
            return (pinLevel == IO::eIOState::LOW_);
        }
        else
        {
            return (pinLevel == IO::eIOState::HIGH_);
        }
    }

  private:
    IO::DigitalInput _io;
    const ePullMode _pullMode;
};

#endif  // ROVER_LIB2_SENSORS_PUSH_BUTTON_HPP
