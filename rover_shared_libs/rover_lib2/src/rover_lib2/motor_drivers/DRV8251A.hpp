#ifndef ROVER_LIB2_MOTOR_DRIVERS_DRV8251A_HPP
#define ROVER_LIB2_MOTOR_DRIVERS_DRV8251A_HPP

#include <algorithm>
#include <cmath>
#include <utility>

#include "rover_lib2/motor_drivers/motor_driver.hpp"
#include "rover_lib2/actuators/PWM_generators/PWM_generator.hpp"

#include "rover_lib2/helpers/macros.hpp"
#include "rover_lib2/helpers/log.hpp"

DEFINE_LOG_NODE(DRV8251A, Logger::eNodeState::OFF);

namespace MotorDrivers
{
    // DRV8251A truth table (INx pins, before PWM inversion):
    //
    //   IN1  IN2  | Function
    //   ----------+---------
    //    0    0   | Brake (both outputs pulled low)
    //    1    0   | Forward (OUT1 high, OUT2 low)
    //    0    1   | Reverse (OUT1 low, OUT2 high)
    //    1    1   | Coast  (Hi-Z, both outputs floating)
    //
    // setDutyCycle(0%)   → pin held LOW
    // setDutyCycle(100%) → pin held HIGH

    template<PWMGenerators::PWMGenerator PwmGenerator1T, PWMGenerators::PWMGenerator PwmGenerator2T>
    class DRV8251A
    {
        static constexpr float FULL_STOP_CMD = 0.0F;
        static constexpr float COAST_STOPPED_ERROR_TOLERANCE = 0.001F;

      public:
        DRV8251A(PwmGenerator1T& pwmA_, PwmGenerator2T& pwmB_, bool reversed_, eBrakeMode brakeMode_ = eBrakeMode::BRAKE):
            _pwmA(pwmA_),
            _pwmB(pwmB_),
            _goalCmd(FULL_STOP_CMD),
            _reversed(reversed_),
            _brakeMode(brakeMode_)
        {
            this->setReversed(_reversed);
            this->setBrakeMode(brakeMode_);
        }

        void init(void)
        {
            _pwmA.init();
            _pwmB.init();

            this->setCmd(FULL_STOP_CMD);
        }

        void update(void)
        {
            switch (_brakeMode)
            {
                case eBrakeMode::BRAKE:
                {
                    // DRV8251A truth table — BRAKE mode:
                    //   Stop    → IN1=0, IN2=0 → active brake
                    //   Forward → IN1=cmd, IN2=0
                    //   Reverse → IN1=0, IN2=cmd

                    // FIX: explicit stop check before direction branch.
                    // Previously `forward = (_goalCmd >= 0)` was true at exactly 0,
                    // so stop accidentally fell into the forward path. While the
                    // result (both at 0%) was numerically correct, it was semantically
                    // wrong and fragile (e.g. if FULL_STOP_CMD ever changes).
                    if (_goalCmd == FULL_STOP_CMD)
                    {
                        LOG_DEBUG(Logger::Nodes::DRV8251A, "BRAKE STOP | A: 0, B: 0");
                        _pwmA.setDutyCycle(FULL_STOP_CMD);  // IN1 = 0 |
                        _pwmB.setDutyCycle(FULL_STOP_CMD);  // IN2 = 0 | → Brake
                    }
                    else if (_goalCmd > FULL_STOP_CMD)
                    {
                        float cmd = std::abs(_goalCmd);
                        LOG_DEBUG(Logger::Nodes::DRV8251A, "BRAKE FWD | A: %f, B: 0", cmd);
                        _pwmA.setDutyCycle(cmd);            // IN1 = PWM |
                        _pwmB.setDutyCycle(FULL_STOP_CMD);  // IN2 = 0   | → Forward
                    }
                    else  // _goalCmd < FULL_STOP_CMD
                    {
                        float cmd = std::abs(_goalCmd);
                        LOG_DEBUG(Logger::Nodes::DRV8251A, "BRAKE REV | A: 0, B: %f", cmd);
                        _pwmA.setDutyCycle(FULL_STOP_CMD);  // IN1 = 0   |
                        _pwmB.setDutyCycle(cmd);            // IN2 = PWM | → Reverse
                    }
                    break;
                }

                case eBrakeMode::COAST:
                {
                    // DRV8251A truth table — COAST mode:
                    //   Stop    → IN1=1, IN2=1 → Hi-Z (coast)
                    //   Forward → IN1=cmd, IN2=0
                    //   Reverse → IN1=0, IN2=cmd
                    //
                    // FIX: same `>= 0` ambiguity as BRAKE mode existed here for the
                    // non-stopped forward/reverse branch. Now uses strict `> 0` / `< 0`.

                    bool stopped = IN_ERROR(_goalCmd, COAST_STOPPED_ERROR_TOLERANCE, FULL_STOP_CMD);

                    if (stopped)
                    {
                        // Both INx HIGH → Hi-Z output (coast)
                        LOG_DEBUG(Logger::Nodes::DRV8251A, "COAST STOP | A: 100, B: 100");
                        _pwmA.setDutyCycle(MAX_CMD_OPEN_LOOP);  // IN1 = 1 |
                        _pwmB.setDutyCycle(MAX_CMD_OPEN_LOOP);  // IN2 = 1 | → Coast
                    }
                    else if (_goalCmd > FULL_STOP_CMD)
                    {
                        float cmd = std::abs(_goalCmd);
                        LOG_DEBUG(Logger::Nodes::DRV8251A, "COAST FWD | A: %f, B: 0", cmd);
                        _pwmA.setDutyCycle(cmd);            // IN1 = PWM |
                        _pwmB.setDutyCycle(FULL_STOP_CMD);  // IN2 = 0   | → Forward
                    }
                    else  // _goalCmd < -COAST_STOPPED_ERROR_TOLERANCE
                    {
                        float cmd = std::abs(_goalCmd);
                        LOG_DEBUG(Logger::Nodes::DRV8251A, "COAST REV | A: 0, B: %f", cmd);
                        _pwmA.setDutyCycle(FULL_STOP_CMD);  // IN1 = 0   |
                        _pwmB.setDutyCycle(cmd);            // IN2 = PWM | → Reverse
                    }
                    break;
                }

                default:
                    _pwmA.setDutyCycle(FULL_STOP_CMD);
                    _pwmB.setDutyCycle(FULL_STOP_CMD);
                    _pwmA.update();
                    _pwmB.update();
                    ASSERT_MSG_ARGS("Unknown brake mode : %u", std::to_underlying(_brakeMode));
            }

            _pwmA.update();
            _pwmB.update();
        }

        // Range [-100; 100]
        void setCmd(float cmd_)
        {
            _goalCmd = CONSTRAIN(cmd_, -_maxCommand, _maxCommand);
            if (_reversed)
            {
                _goalCmd = -_goalCmd;
            }
        }

        void setEnabled(bool on_)
        {
            // TODO: wire up nSLEEP / nEN pin when hardware supports it.
            // if (_enabled == on_) { return; }
            // _enabled = on_;
            // _ioNotEn.write(on_ ? IO::eIOState::LOW_ : IO::eIOState::HIGH_);
            (void)on_;
        }

        bool isEnabled(void) const
        {
            return true;
        }

        float getCmd(void) const
        {
            return _reversed ? -_goalCmd : _goalCmd;
        }

        void setReversed(bool reversed_)
        {
            _reversed = reversed_;
            this->setCmd(this->getCmd());
        }

        bool isReversed(void) const
        {
            return _reversed;
        }

        void setBrakeMode(eBrakeMode mode_)
        {
            _brakeMode = mode_;
        }

        eBrakeMode getBrakeMode(void) const
        {
            return _brakeMode;
        }

        void setMaxCmd(float cmd_)
        {
            float newMax = std::abs(cmd_);
            newMax = std::clamp(newMax, 0.0F, MotorDrivers::MAX_CMD_OPEN_LOOP);
            _maxCommand = newMax;
            this->setCmd(this->getCmd());
        }

        void setMaxVoltage(float alim_, float maxVoltage_)
        {
            float absAlim = std::abs(alim_);
            float absMaxVoltage = std::clamp(std::abs(maxVoltage_), 0.0F, absAlim);
            this->setMaxCmd((absMaxVoltage / absAlim) * MotorDrivers::MAX_CMD_OPEN_LOOP);
        }

      private:
        PwmGenerator1T& _pwmA;
        PwmGenerator2T& _pwmB;

        float _maxCommand = MotorDrivers::MAX_CMD_OPEN_LOOP;
        float _goalCmd;
        bool _reversed;
        eBrakeMode _brakeMode;

        VALIDATE_CONCEPT(MotorDriver, DRV8251A);
    };

    // Deduction guide — matches constructor signature exactly
    template<typename PwmGenerator1T, typename PwmGenerator2T>
    DRV8251A(PwmGenerator1T&, PwmGenerator2T&, bool, eBrakeMode) -> DRV8251A<PwmGenerator1T, PwmGenerator2T>;

}  // namespace MotorDrivers

#endif  // ROVER_LIB2_MOTOR_DRIVERS_DRV8251A_HPP