#ifndef ROVER_LIB2_MOTOR_DRIVERS_IFX007T_HPP
#define ROVER_LIB2_MOTOR_DRIVERS_IFX007T_HPP

#include "rover_lib2/motor_drivers/motor_driver.hpp"
#include "rover_lib2/actuators/PWM_generators/PWM_generator.hpp"
#include "rover_lib2/IO/digital_output.hpp"

#include "rover_lib2/helpers/macros.hpp"
#include "rover_lib2/helpers/log.hpp"

DEFINE_LOG_NODE(IFX007T, Logger::eNodeState::OFF);

namespace MotorDrivers
{
    template<PWMGenerators::PWMGenerator PwmGeneratorAT, PWMGenerators::PWMGenerator PwmGeneratorBT>
    class IFX007T
    {
        // Error tolerated in _goalCmd to consider full stop in coast
        static constexpr float COAST_STOPPED_ERROR_TOLERANCE = 0.001F;
        static constexpr float FULL_STOP_CMD = 0.0F;

      public:
        IFX007T(IO::DigitalOutput& enableA_,
                PwmGeneratorAT& pwmA_,
                IO::DigitalOutput& enableB_,
                PwmGeneratorBT& pwmB_,
                bool reversed_,
                eBrakeMode brakeMode_ = eBrakeMode::BRAKE):
            _enableA(enableA_),
            _pwmA(pwmA_),
            _enableB(enableB_),
            _pwmB(pwmB_),
            _goalCmd(0),
            _reversed(reversed_),
            _enabled(false),
            _brakeMode(brakeMode_)
        {
            this->setReversed(_reversed);
            this->setEnabled(_enabled);
            this->setBrakeMode(brakeMode_);

            ASSERT_COND_MSG(
                pwmA_.getFrequency() <= 1'000.0F && pwmB_.getFrequency() <= 1'000.0F,
                "IFX007T drivers are quite bad and can't handle pwm frequency over 1'000Hz without loosing precision and "
                "generating a lot of jitter and back EMF");
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
                    float cmd = ABS(_goalCmd);
                    bool forward = (_goalCmd >= FULL_STOP_CMD);

                    if (forward)
                    {
                        LOG_DEBUG(Logger::Nodes::IFX007T, "BRAKE FWD | A: %f, B: %f", cmd, FULL_STOP_CMD);
                        _pwmA.setDutyCycle(cmd);
                        _pwmB.setDutyCycle(FULL_STOP_CMD);
                    }
                    else
                    {
                        LOG_DEBUG(Logger::Nodes::IFX007T, "BRAKE FWD | A: %f, B: %f", FULL_STOP_CMD, cmd);
                        _pwmA.setDutyCycle(FULL_STOP_CMD);
                        _pwmB.setDutyCycle(cmd);
                    }
                    break;
                }
                case eBrakeMode::COAST:
                {
                    bool stopped = IN_ERROR(_goalCmd, COAST_STOPPED_ERROR_TOLERANCE, FULL_STOP_CMD);
                    if (stopped)
                    {
                        this->setEnabled(false);
                    }
                    else
                    {
                        this->setEnabled(true);
                    }

                    float cmd = ABS(_goalCmd);
                    bool forward = (_goalCmd >= FULL_STOP_CMD);
                    if (forward)
                    {
                        _pwmA.setDutyCycle(cmd);
                        _pwmB.setDutyCycle(FULL_STOP_CMD);
                    }
                    else
                    {
                        _pwmA.setDutyCycle(FULL_STOP_CMD);
                        _pwmB.setDutyCycle(cmd);
                    }
                    break;
                }
                default:
                    _pwmA.setDutyCycle(FULL_STOP_CMD);
                    _pwmB.setDutyCycle(FULL_STOP_CMD);
                    _pwmA.update();
                    _pwmB.update();
                    ASSERT_MSG_ARGS("Unknown brake mode : %u, falling in error mode", std::to_underlying(_brakeMode));
            }

            _pwmA.update();
            _pwmB.update();
        }

        // Range is [-100; 100]
        void setCmd(float cmd_)
        {
            _goalCmd = CONSTRAIN(cmd_, -_maxCommand, _maxCommand);
            if (_reversed)
            {
                _goalCmd = -_goalCmd;
            }
        }

        // Range is [-100; 100]
        float getCmd(void) const
        {
            if (_reversed)
            {
                return -_goalCmd;
            }
            else
            {
                return _goalCmd;
            }
        }

        void setReversed(bool reversed_)
        {
            _reversed = reversed_;
            this->setCmd(this->getCmd());  // will apply reversed on the internal command value
        }

        bool isReversed(void) const
        {
            return _reversed;
        }

        void setEnabled(bool on_)
        {
            if (_enabled == on_)
            {
                return;
            }

            _enabled = on_;
            if (_enabled)
            {
                _enableA.write(IO::eIOState::HIGH_);
                _enableB.write(IO::eIOState::HIGH_);
            }
            else
            {
                _enableA.write(IO::eIOState::LOW_);
                _enableB.write(IO::eIOState::LOW_);
            }
        }

        bool isEnabled(void) const
        {
            return _enabled;
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
            newMax = std::clamp(cmd_, 0.0F, MotorDrivers::MAX_CMD_OPEN_LOOP);

            _maxCommand = newMax;
            this->setCmd(this->getCmd());
        }

        void setMaxVoltage(float alim_, float maxVoltage_)
        {
            float absAlim = std::abs(alim_);
            float absMaxVotlage = std::abs(maxVoltage_);

            absMaxVotlage = std::clamp(absMaxVotlage, 0.0F, absAlim);

            float newCmd = (absMaxVotlage / absAlim) * MotorDrivers::MAX_CMD_OPEN_LOOP;
            this->setMaxCmd(newCmd);
        }

      private:
        IO::DigitalOutput& _enableA;
        PwmGeneratorAT& _pwmA;
        IO::DigitalOutput& _enableB;
        PwmGeneratorBT& _pwmB;

        float _maxCommand = MotorDrivers::MAX_CMD_OPEN_LOOP;

        float _goalCmd;
        bool _reversed;
        bool _enabled;
        eBrakeMode _brakeMode;

        VALIDATE_CONCEPT(MotorDriver, IFX007T);
    };

    template<typename PwmGeneratorAT, typename PwmGeneratorBT>
    IFX007T(IO::DigitalOutput&, PwmGeneratorAT&, IO::DigitalOutput&, PwmGeneratorBT&, bool, eBrakeMode)
        -> IFX007T<PwmGeneratorAT, PwmGeneratorBT>;

}  // namespace MotorDrivers

#endif  // ROVER_LIB2_MOTOR_DRIVERS_IFX007T_HPP
