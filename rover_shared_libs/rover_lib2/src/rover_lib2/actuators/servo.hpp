#ifndef ROVER_LIB2_ACTUATORS_SERVO_HPP
#define ROVER_LIB2_ACTUATORS_SERVO_HPP

#include "actuator.hpp"

#include <rover_lib2/helpers/assert.hpp>
#include <rover_lib2/actuators/PWM_generators/PWM_generator.hpp>
#include <rover_lib2/helpers/time.hpp>
#include <rover_lib2/helpers/loop_timer.hpp>
#include <rover_lib2/helpers/macros.hpp>

DEFINE_LOG_NODE(ActuatorServo, Logger::eNodeState::OFF);

namespace Actuators
{
    class ServoT
    {
      protected:
        ServoT() = default;

      public:
        struct sTimingConfig
        {
            float frequency;
            float minMs;
            float maxMs;
            float minPosition;
            float maxPosition;
            float maxSpeed;  // Used to simulate encoder during movement
        };
    };

    template<PWMGenerators::PWMGenerator PwmGeneratorT>
    class Servo
    {
        static constexpr float UPDATE_FREQUENCY = 1'000.0F;
        static constexpr uint64_t UPDATE_PERIOD = 1'000ULL / static_cast<uint64_t>(UPDATE_FREQUENCY);

        static constexpr float SERVO_FREQUENCY_TOLERANCE_HZ = 10.0F;  // +- Hz

      public:
        Servo(ServoT::sTimingConfig servoTimings_,
              PwmGeneratorT& pwmGenerator_,
              bool reversed_ = false,
              float initialPos_ = 0.0F):
            _servoTimings(servoTimings_),
            _pwmGenerator(pwmGenerator_),
            _reversed(reversed_),
            _minPosition(_servoTimings.minPosition),
            _maxPosition(_servoTimings.maxPosition),
            _maxSpeed(_servoTimings.maxSpeed),
            _msToDutyFactor(1.0F / (1'000'000.0F / _servoTimings.frequency)),
            _updateTimer(UPDATE_PERIOD)
        {
            ASSERT_COND(IN_ERROR(_pwmGenerator.getFrequency(), SERVO_FREQUENCY_TOLERANCE_HZ, _servoTimings.frequency));

            this->setJointLimit(_servoTimings.minPosition, _servoTimings.maxPosition);

            _goalPos = CONSTRAIN(initialPos_, _minPosition, _maxPosition);
            _currentPos = _goalPos + 1.0E-6F;  // Creating a very small offset otherwise the update will return early
        }

        void init(void)
        {
            _pwmGenerator.init();
            _pwmGenerator.setEnabled(true);
        }

        void update(void)
        {
            if (_updateTimer.isReady())
            {
                LOG_DEBUG(Logger::Nodes::ActuatorServo, "_currentPos: %f, _goalPos: %f", _currentPos, _goalPos);
                if (_currentPos != _goalPos)
                {
                    float enlapsedS = static_cast<float>(_enlapsedSinceLastUpdate.getTime()) / 1'000'000.0F;
                    float maxIncrement = enlapsedS * _maxSpeed;

                    float cmd = _goalPos;
                    if ((_currentPos <= _goalPos))
                    {
                        if (_currentPos + maxIncrement >= _goalPos)
                        {
                            cmd = _goalPos;
                        }
                        else
                        {
                            cmd = _currentPos + maxIncrement;
                        }
                    }
                    else
                    {
                        if (_currentPos - maxIncrement <= _goalPos)
                        {
                            cmd = _goalPos;
                        }
                        else
                        {
                            cmd = _currentPos - maxIncrement;
                        }
                    }

                    _currentPos = cmd;
                    float minDuty = _servoTimings.minMs * _msToDutyFactor;
                    float maxDuty = _servoTimings.maxMs * _msToDutyFactor;
                    cmd = MAP(cmd, _servoTimings.minPosition, _servoTimings.maxPosition, minDuty, maxDuty);
                    _pwmGenerator.setDutyCycle(100.0F * cmd);
                }

                _enlapsedSinceLastUpdate.restart();
                _pwmGenerator.update();
            }
        }

        void setPosition(float pos_)
        {
            pos_ = CONSTRAIN(pos_, _minPosition, _maxPosition);
            if (pos_ == this->getPosition())
            {
                return;
            }

            if (!_reversed)
            {
                _goalPos = pos_;
            }
            else
            {
                if (_minPosition == _maxPosition)
                {
                    _goalPos = _minPosition;
                }
                else
                {
                    _goalPos = MAP(pos_,
                                   _servoTimings.minPosition,
                                   _servoTimings.maxPosition,
                                   _servoTimings.maxPosition,
                                   _servoTimings.minPosition);
                }
            }
        }

        float getPosition(void) const
        {
            if (!_reversed)
            {
                return _currentPos;
            }
            else
            {
                return MAP(_currentPos,
                           _servoTimings.minPosition,
                           _servoTimings.maxPosition,
                           _servoTimings.maxPosition,
                           _servoTimings.minPosition);
            }
        }

        void setSpeed(float speed_)
        {
            ASSERT_MSG("Not supported");
        }

        float getSpeed(void) const
        {
            if (_goalPos == _currentPos)
            {
                return 0.0F;
            }
            else if (_goalPos > _currentPos)
            {
                return -_maxSpeed;
            }
            else
            {
                return _maxSpeed;
            }
        }

        void setMaxSpeed(float max_speed_)
        {
            max_speed_ = std::abs(max_speed_);
            _maxSpeed = CONSTRAIN(max_speed_, 0.0F, _servoTimings.maxSpeed);
        }

        /**
         * @brief Not supported on Servo actuators
         *
         */
        void calib(float)
        {
            // Not supported
        }

        void setJointLimit(std::optional<float> min_, std::optional<float> max_)
        {
            if (!min_ && !max_)
            {
                return;
            }

            if (min_ && _minPosition != CONSTRAIN(min_.value(), _servoTimings.minPosition, _servoTimings.maxPosition))
            {
                _minPosition = CONSTRAIN(min_.value(), _servoTimings.minPosition, _servoTimings.maxPosition);
            }
            else
            {
                _minPosition = _servoTimings.minPosition;
            }

            if (max_ && _maxPosition != CONSTRAIN(max_.value(), _servoTimings.minPosition, _servoTimings.maxPosition))
            {
                LOG_INFO(Logger::Nodes::ActuatorServo, "Here");
                _maxPosition = CONSTRAIN(max_.value(), _servoTimings.minPosition, _servoTimings.maxPosition);
            }
            else
            {
                _maxPosition = _servoTimings.maxPosition;
            }

            if (_minPosition > _maxPosition)
            {
                ASSERT_COND_MSG_ARGS(_minPosition <= _maxPosition,
                                     "Can't set min joint limit (%f) higher than max joint limits (%f), stopping joint",
                                     _minPosition,
                                     _maxPosition);

                _minPosition = _maxPosition;
            }
        }

      private:
        const ServoT::sTimingConfig _servoTimings;
        PwmGeneratorT& _pwmGenerator;

        float _goalPos = 0.0F;
        float _currentPos = 0.0F;
        const bool _reversed = false;

        float _minPosition;
        float _maxPosition;
        float _maxSpeed;

        const float _msToDutyFactor;
        Chrono<uint64_t, Time::micros> _enlapsedSinceLastUpdate;
        LoopTimer<uint64_t, Time::millis> _updateTimer;

        VALIDATE_CONCEPT(Actuator, Servo);
    };

}  // namespace Actuators

#endif  // ROVER_LIB2_ACTUATORS_SERVO_HPP
