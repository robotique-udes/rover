#ifndef IFX007T_HPP
#define IFX007T_HPP

#include "rover_lib2/actuators/motor_drivers/motor_driver.hpp"
#include "rover_lib2/actuators/PWM_generators/PWM_generator.hpp"
#include "rover_lib2/IO/digital_output.hpp"

#include "rover_lib2/helpers/macros.hpp"
#include "rover_lib2/helpers/log.hpp"

DEFINE_LOG_NODE(IFX007T, Logger::eNodeState::OFF);

template<typename PwmGeneratorAT, typename PwmGeneratorBT>
class IFX007T : public MotorDriver<IFX007T<PwmGeneratorAT, PwmGeneratorBT>>
{
    VALIDATE_BASE_TYPE(PWMGenerators::PWMGeneratorT, PwmGeneratorAT);
    VALIDATE_BASE_TYPE(PWMGenerators::PWMGeneratorT, PwmGeneratorBT);

    // Error tolerated in _goalCmd to consider full stop in coast
    static constexpr float COAST_STOPPED_ERROR_TOLERANCE = 0.001F;
    static constexpr float FULL_STOP_CMD = 0.0F;

  public:
    IFX007T(IO::DigitalOutput& enableA_,
            PwmGeneratorAT& pwmA_,
            IO::DigitalOutput& enableB_,
            PwmGeneratorBT& pwmB_,
            bool reversed_,
            MotorDriverT::eBrakeMode brakeMode_ = MotorDriverT::eBrakeMode::BRAKE):
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
    }

    void __init(void)
    {
        _pwmA.init();
        _pwmB.init();

        this->setCmd(FULL_STOP_CMD);
    }

    void __update(void)
    {
        switch (_brakeMode)
        {
            case MotorDriverT::eBrakeMode::BRAKE:
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
            case MotorDriverT::eBrakeMode::COAST:
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
        }

        _pwmA.update();
        _pwmB.update();
    }

    // Range is [-100; 100]
    void _setCmd(float cmd_)
    {
        _goalCmd = CONSTRAIN(cmd_, -100.0F, 100.0F);
        if (_reversed)
        {
            _goalCmd = -_goalCmd;
        }
    }

    // Range is [-100; 100]
    float _getCmd(void)
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
        this->setCmd(this->getCmd());  // will apply reversed on the inernal command value
    }

    bool isReversed(void)
    {
        return _reversed;
    }

    void _setEnabled(bool on_)
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

    bool _isEnabled(void)
    {
        return _enabled;
    }

    void _setBrakeMode(MotorDriverT::eBrakeMode mode_)
    {
        _brakeMode = mode_;
    }

    MotorDriverT::eBrakeMode _getBrakeMode(void)
    {
        return _brakeMode;
    }

  private:
    IO::DigitalOutput& _enableA;
    PwmGeneratorAT& _pwmA;
    IO::DigitalOutput& _enableB;
    PwmGeneratorBT& _pwmB;

    float _goalCmd;
    bool _reversed;
    bool _enabled;
    MotorDriverT::eBrakeMode _brakeMode;
};

template<typename PwmGeneratorAT, typename PwmGeneratorBT>
IFX007T(IO::DigitalOutput&, PwmGeneratorAT&, IO::DigitalOutput&, PwmGeneratorBT&, bool, MotorDriverT::eBrakeMode)
    -> IFX007T<PwmGeneratorAT, PwmGeneratorBT>;

#endif  // IFX007T_HPP
