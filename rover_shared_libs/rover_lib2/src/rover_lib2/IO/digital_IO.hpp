#ifndef DIGITAL_IO_HPP
#define DIGITAL_IO_HPP

#include <rover_lib2/rover_object.hpp>
#include <rover_lib2/helpers/log.hpp>
#include <rover_lib2/helpers/assert.hpp>

#if defined(ARDUINO_ESP32S3_DEV)
#include "driver/gpio.h"
#include "soc/gpio_struct.h"

DEFINE_LOG_NODE(DigitalIO, Logger::eNodeState::ON);

namespace IO
{
    enum class eIOState : uint32_t
    {
        LOW_ = 0U,
        HIGH_ = 1U,
    };

    class DigitalOutput
    {
        static constexpr gpio_num_t GPIO_DIRECT_ACCESS_MAX = static_cast<gpio_num_t>(31);

      public:
        explicit DigitalOutput(gpio_num_t pin_,
                               eIOState initialState_ = eIOState::LOW_,
                               gpio_mode_t mode_ = gpio_mode_t::GPIO_MODE_OUTPUT,
                               gpio_pull_mode_t pullMode_ = gpio_pull_mode_t::GPIO_FLOATING,
                               gpio_drive_cap_t powerMode_ = gpio_drive_cap_t::GPIO_DRIVE_CAP_DEFAULT):
            _pin(pin_),
            _isDirectAccess(_pin <= GPIO_DIRECT_ACCESS_MAX)
        {
            this->setMode(mode_);
            this->setPullMode(pullMode_);
            this->setPowerMode(powerMode_);

            this->write(initialState_);
        }

        void write(eIOState state_)
        {
            switch (state_)
            {
                case eIOState::HIGH_:
                    if (_isDirectAccess)
                    {
                        GPIO.out_w1ts = (1 << _pin);
                    }
                    else
                    {
                        gpio_set_level(_pin, static_cast<uint32_t>(state_));
                    }
                    break;
                case eIOState::LOW_:
                    if (_isDirectAccess)
                    {
                        GPIO.out_w1tc = (1 << _pin);
                    }
                    else
                    {
                        gpio_set_level(_pin, static_cast<uint32_t>(state_));
                    }
                    break;
                default:
                    ASSERT(false, "Invalid argument | Tried to write gpio state: %u", static_cast<uint32_t>(state_));
                    break;
            }
            return;
        }

        eIOState read(void)
        {
            if (_isDirectAccess)
            {
                LOG_DEBUG(Logger::Nodes::DigitalIO, "Pin state: %d", GPIO.out & (1 << _pin));
                bool pinHigh = static_cast<bool>(GPIO.out & (1 << _pin));
                eIOState pinState = pinHigh ? eIOState::HIGH_ : eIOState::LOW_;
                return pinState;
            }
            else
            {
                return static_cast<eIOState>(gpio_get_level(_pin));
            }
        }

        void toggle(void)
        {
            if (_isDirectAccess)
            {
                GPIO.out = GPIO.out ^ (1 << _pin);
            }
            else
            {
                this->write(this->read());
            }
        }

        void setMode(gpio_mode_t mode_)
        {
            ASSERT(mode_ != gpio_mode_t::GPIO_MODE_INPUT_OUTPUT || mode_ != gpio_mode_t::GPIO_MODE_INPUT_OUTPUT_OD
                       || mode_ != gpio_mode_t::GPIO_MODE_OUTPUT || mode_ != gpio_mode_t::GPIO_MODE_OUTPUT_OD,
                   "Wrong mode selected for DigitalIO, implementation error. Undefined behavior on IO");
            gpio_set_direction(_pin, mode_);
        }

        void setPullMode(gpio_pull_mode_t pullMode_)
        {
            gpio_set_pull_mode(_pin, pullMode_);
        }

        void setPowerMode(gpio_drive_cap_t powerMode_)
        {
            gpio_set_drive_capability(_pin, powerMode_);
        }

      private:
        const gpio_num_t _pin;
        const bool _isDirectAccess;
    };
}  // namespace IO

#endif  // defined(ARDUINO_ESP32S3_DEV)

#endif  // DIGITAL_IO_HPP
