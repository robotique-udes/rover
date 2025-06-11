#ifndef DIGITAL_INPUT_HPP
#define DIGITAL_INPUT_HPP

#include "rover_lib2/rover_object.hpp"
#include "rover_lib2/helpers/log.hpp"
#include "rover_lib2/helpers/assert.hpp"
#include "rover_lib2/IO/digital_io.hpp"

#if defined(ARDUINO_ESP32S3_DEV)
#include "driver/gpio.h"
#include "soc/gpio_struct.h"

DEFINE_LOG_NODE(DigitalInput, Logger::eNodeState::OFF);

namespace IO
{
    class DigitalInput
    {
        static constexpr gpio_num_t GPIO_DIRECT_ACCESS_MAX = static_cast<gpio_num_t>(31);

      public:
        explicit DigitalInput(gpio_num_t pin_,
                              gpio_mode_t mode_ = gpio_mode_t::GPIO_MODE_INPUT,
                              gpio_pull_mode_t pullMode_ = gpio_pull_mode_t::GPIO_FLOATING,
                              gpio_drive_cap_t powerMode_ = gpio_drive_cap_t::GPIO_DRIVE_CAP_0):
            _pin(pin_),
            _isDirectAccess(_pin <= GPIO_DIRECT_ACCESS_MAX)
        {
            if (_pin != GPIO_NUM_NC)
            {
                gpio_reset_pin(pin_);
                this->setMode(mode_);
                this->setPullMode(pullMode_);
            }
        }

        eIOState read(void)
        {
            if (_pin == GPIO_NUM_NC)
            {
                return;
            }

            if (_isDirectAccess)
            {
                LOG_DEBUG(Logger::Nodes::DigitalInput, "Pin state: %d", GPIO.in & (1 << _pin));
                bool pinHigh = static_cast<bool>(GPIO.in & (1 << _pin));
                eIOState pinState = pinHigh ? eIOState::HIGH_ : eIOState::LOW_;
                return pinState;
            }
            else
            {
                return static_cast<eIOState>(gpio_get_level(_pin));
            }
        }

        void setMode(gpio_mode_t mode_)
        {
            if (_pin == GPIO_NUM_NC)
            {
                return;
            }

            ASSERT_COND_MSG(mode_ == gpio_mode_t::GPIO_MODE_INPUT_OUTPUT || mode_ == gpio_mode_t::GPIO_MODE_INPUT_OUTPUT_OD
                                || mode_ == gpio_mode_t::GPIO_MODE_OUTPUT || mode_ == gpio_mode_t::GPIO_MODE_OUTPUT_OD,
                            "Wrong mode selected for DigitalIO, implementation error. Undefined behavior on IO");
            gpio_set_direction(_pin, mode_);
        }

        void setPullMode(gpio_pull_mode_t pullMode_)
        {
            if (_pin == GPIO_NUM_NC)
            {
                return;
            }

            gpio_set_pull_mode(_pin, pullMode_);
        }

      private:
        const gpio_num_t _pin;
        const bool _isDirectAccess;
    };
}  // namespace IO

#endif  // defined(ARDUINO_ESP32S3_DEV)

#endif  // DIGITAL_INPUT_HPP
