#ifndef ROVER_LIB2_LED_LED_BLINKER_HPP
#define ROVER_LIB2_LED_LED_BLINKER_HPP

#include "rover_lib2/LED/blink_pattern.hpp"

#include "rover_lib2/IO/digital_output.hpp"
#include "rover_lib2/rover_object.hpp"
#include "rover_lib2/helpers/one_shot_timer.hpp"
#include "rover_lib2/helpers/loop_timer.hpp"
#include "rover_lib2/helpers/macros.hpp"
#include "rover_lib2/helpers/time.hpp"

DEFINE_LOG_NODE(LedBlinker, Logger::eNodeState::ON);

namespace LED
{
    class LedBlinkerT
    {
      protected:
        LedBlinkerT() = default;
    };

    template<typename ImplT>
    class LedBlinker : public LedBlinkerT
    {
      public:
        void init(void)
        {
            static_cast<ImplT*>(this)->__init();
        }

        void update(void)
        {
            static_cast<ImplT*>(this)->__update();
        }

        template<size_t STEP_CTN>
        void setPattern(const BlinkingPattern<STEP_CTN>& pattern_)
        {
            static_cast<ImplT*>(this)->_setPattern(pattern_);
        }

      private:
        friend ImplT;
        LedBlinker() = default;
    };

    class LedBlinkerSoft : public LedBlinker<LedBlinkerSoft>
    {
        enum class eState
        {
            ON,
            DIMMED,
            OFF
        };

      public:
        template<size_t STEP_CTN>
        LedBlinkerSoft(IO::DigitalOutput led_,
                       const BlinkingPattern<STEP_CTN>& pattern_,
                       uint8_t maxIntensity_ = LED::INTENSITY_RESOLUTION,
                       uint32_t ledFrequency_ = 50UL):
            LedBlinkerSoft(led_, pattern_.data(), pattern_.size(), maxIntensity_, ledFrequency_)
        {
        }

        void __init(void) {}

        void __update(void)
        {
            if (!_pattern)
            {
                return;
            }

            if (_currentLedState == eState::DIMMED && _timerIntensity.isReady())
            {
                triggerCtn++;
                if (triggerCtn == triggerCtnTarget)
                {
                    _led.write(IO::eIOState::LOW_);
                }
                else if (triggerCtn == INTENSITY_RESOLUTION)
                {
                    _led.write(IO::eIOState::HIGH_);
                    triggerCtn = 0U;
                }
            }
            else if (_chronoStep.getTime() >= _targetTimeStep)
            {
                this->toggleNextStep();
            }
        }

        template<size_t STEP_CTN>
        void setPattern(const BlinkingPattern<STEP_CTN>& pattern_)
        {
            this->setPattern(pattern_.data(), pattern_.size());
        }

      private:
        LedBlinkerSoft(IO::DigitalOutput led_,
                       const BlinkPatternStep* pattern_,
                       size_t patternSize_,
                       uint8_t maxIntensity_ = LED::INTENSITY_RESOLUTION,
                       uint32_t ledFrequency_ = 50UL):
            _led(led_),
            _timerIntensity(1'000'000UL / (ledFrequency_ * 2UL * INTENSITY_RESOLUTION)),
            _targetTimeStep(0UL),
            _maxIntensity(maxIntensity_)
        {
            this->setPattern(pattern_, patternSize_);
        }

        void setPattern(const BlinkPatternStep* pattern_, size_t patternSize_)
        {
            if (_pattern == pattern_)
            {
                return;
            }

            ASSERT_COND_MSG(pattern_, "Passed pattern can't be null");
            ASSERT_COND_MSG(patternSize_ > 0, "Pattern size must be at least 1");

            _pattern = pattern_;
            _patternSize = patternSize_;
            _currentStep = 0UL;
            _targetTimeStep = 0UL;
        }

        void toggleNextStep(void)
        {
            if (_pattern)
            {
                _currentStep = (_currentStep + 1UL) == _patternSize ? 0UL : _currentStep + 1UL;

                _targetTimeStep = static_cast<uint64_t>(_pattern[_currentStep].durationMs);
                _chronoStep.restart();

                uint8_t stepIntensity = CONSTRAIN(_pattern[_currentStep].intensity, static_cast<uint8_t>(0), _maxIntensity);

                if (stepIntensity == 0)
                {
                    _currentLedState = eState::OFF;
                    _led.write(IO::eIOState::LOW_);
                }
                else if (stepIntensity == INTENSITY_RESOLUTION)
                {
                    _currentLedState = eState::ON;
                    _led.write(IO::eIOState::HIGH_);
                }
                else
                {
                    _currentLedState = eState::DIMMED;
                    triggerCtnTarget = CONSTRAIN(stepIntensity, static_cast<uint8_t>(0), INTENSITY_RESOLUTION);
                    triggerCtn = 0UL;
                    _led.write(IO::eIOState::HIGH_);
                }
            }
        }

        IO::DigitalOutput _led;
        const BlinkPatternStep* _pattern = nullptr;
        size_t _patternSize;
        size_t _currentStep = 0UL;

        eState _currentLedState = eState::OFF;
        LoopTimer<uint64_t, Time::micros> _timerIntensity;
        uint8_t triggerCtnTarget;
        uint8_t triggerCtn;

        Chrono<uint64_t, Time::millis> _chronoStep;
        uint64_t _targetTimeStep;
        uint8_t _maxIntensity;
    };
}  // namespace LED

#endif  // ROVER_LIB2_LED_LED_BLINKER_HPP
