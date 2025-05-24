#ifndef ACTUATOR_SERVO_HPP
#define ACTUATOR_SERVO_HPP

#error IMPLEMENTATION IN PROGRESS DON'T USE

#include "actuator.hpp"
#include "rover_lib2/sensors/encoder/encoder.hpp"
#include "motor_drivers/motor_driver.hpp"
#include "rover_lib2/helpers/macros.hpp"
#include "rover_lib2/helpers/assert.hpp"
#include "rover_lib2/helpers/constants.hpp"
#include "rover_lib2/actuators/PWM_generators/PWM_generator.hpp"
#include "rover_lib2/helpers/chrono.hpp"
#include "rover_lib2/helpers/time.hpp"
#include "rover_lib2/helpers/loop_timer.hpp"

#include <array>

class ActuatorServoT
{
  protected:
    ActuatorServoT() = default;

  public:
    enum class eModel
    {
        BILDA_TORQUE_FIVE_TURN,
        eLast
    };

    struct sTimingLimits
    {
        float frequency;
        float minMs;
        float maxMs;
        float minPosition;
        float maxPosition;
        float maxSpeed;  // Used to simulate position during movement
    };

    static constexpr std::array<sTimingLimits, TO_UNDERLYING(eModel::eLast)> TIMING_CONFIGS
        = {sTimingLimits{.frequency = 50.0F,
                         .minMs = 1000.0F,
                         .maxMs = 2000.0F,
                         .minPosition = 0.0F,
                         .maxPosition = Constants::TWO_PI_,
                         .maxSpeed = 1000.0F}};
};

template<typename PwmGenerator_T>
class ActuatorServo : public Actuator<ActuatorServo<PwmGenerator_T>>
{
    VALIDATE_BASE_TYPE(PWMGenerators::PWMGeneratorT, PwmGenerator_T);
    static constexpr float UPDATE_FREQUENCY = 1'000.0F;
    static constexpr uint64_t UPDATE_PERIOD = 1'000ULL / static_cast<uint64_t>(UPDATE_FREQUENCY);

  public:
    ActuatorServo(ActuatorServoT::eModel model_, PwmGenerator_T& pwmGenerator_, bool reversed_ = false):
        _model(model_),
        _pwmGenerator(pwmGenerator_),
        _msToDutyFactor(1.0F / (1'000'000.0F / ActuatorServoT::TIMING_CONFIGS[TO_UNDERLYING(_model)].frequency)),
        _updateTimer(UPDATE_PERIOD)
    {
        ASSERT_COND(TO_UNDERLYING(_model) < TO_UNDERLYING(ActuatorServoT::eModel::eLast));
        ASSERT_COND(
            IN_ERROR(_pwmGenerator.getFrequency(), 1.0F, ActuatorServoT::TIMING_CONFIGS[TO_UNDERLYING(_model)].frequency));

        _minPosition = ActuatorServoT::TIMING_CONFIGS[TO_UNDERLYING(_model)].minPosition;
        _maxPosition = ActuatorServoT::TIMING_CONFIGS[TO_UNDERLYING(_model)].maxPosition;

        this->setReversed(reversed_);
    }

    void __init(void)
    {
        _pwmGenerator.init();
    }

    void __update(void)
    {
        if (_updateTimer.isReady())
        {
            if (_currentPos != _goalPos)
            {
                float enlapseS = static_cast<float>(_enlapseSinceLastUpdate.getTime()) / 1'000'000.0F;
                float maxIncrement = enlapseS * _maxSpeed;

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

                float minDuty = ActuatorServoT::TIMING_CONFIGS[TO_UNDERLYING(_model)].minMs * _msToDutyFactor;
                float maxDuty = ActuatorServoT::TIMING_CONFIGS[TO_UNDERLYING(_model)].maxMs * _msToDutyFactor;
                cmd = MAP(_goalPos, _minPosition, _maxPosition, minDuty, maxDuty);
                _pwmGenerator.setDutyCycle(100.0F * cmd);
            }

            _enlapseSinceLastUpdate.restart();
            _pwmGenerator.update();
        }
    }

    void _setPosition(float pos_)
    {
        CONSTRAIN(pos_, _minPosition, _maxPosition);
        _goalPos = pos_;
    }

    float _getPosition(void)
    {
        return _currentPos;
    }

    void _setSpeed(float speed_)
    {
        ASSERT_MSG("Not supported");
    }

    float _getSpeed(void)
    {
        if (_goalPos == _currentPos)
        {
            return 0.0F;
        }
        else
        {
            return _maxSpeed;
        }
    }

    void _setMaxSpeed(float max_speed_)
    {
        ASSERT_MSG("TODO");
    }

    void _setJointLimit(float min_, float max_)
    {
        ASSERT_COND_MSG(min_ <= max_, "Y'a dumb biatch");

        _minPosition = min_;
        _maxPosition = max_;

        _goalPos = CONSTRAIN(_goalPos, _minPosition, _maxPosition);
    }

    void _setReversed(bool reversed_)
    {
        _reversed = reversed_;
    }

  private:
    const ActuatorServoT::eModel _model;
    PwmGenerator_T& _pwmGenerator;

    float _goalPos = 0.0F;
    float _currentPos = 0.0F;
    bool _reversed = false;

    float _minPosition = 0.0F;
    float _maxPosition = 0.0F;
    float _maxSpeed = 0.0F;

    const float _msToDutyFactor;
    Chrono<uint64_t, Time::micros> _enlapseSinceLastUpdate;
    LoopTimer<uint64_t, Time::millis> _updateTimer;
};

#endif  // ACTUATOR_SERVO_HPP
