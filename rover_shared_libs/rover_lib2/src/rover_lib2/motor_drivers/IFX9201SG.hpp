#ifndef ROVER_LIB2_MOTOR_DRIVERS_IFX9201SG_HPP
#define ROVER_LIB2_MOTOR_DRIVERS_IFX9201SG_HPP

#include "motor_driver.hpp"

#include "rover_lib2/helpers/assert.hpp"
#include "rover_lib2/IO/digital_output.hpp"

#include "rover_lib2/actuators/PWM_generators/PWM_generator.hpp"

#include <optional>

DEFINE_LOG_NODE(IFX9201SG, Logger::eNodeState::OFF);

namespace MotorDrivers
{

    template<PWMGenerators::PWMGenerator PWMGeneratorT>
    class IFX9201SG
    {
        static constexpr float ZERO_ERROR_TOLERANCE
            = 0.001F;  // When command are considered null, mustn't move nor heat the driver/motor

      public:
        IFX9201SG(PWMGeneratorT& pwmGenerator_,
                  gpio_num_t pinDir_,
                  bool reversed_,
                  gpio_num_t pinDis_ = GPIO_NUM_NC,
                  eBrakeMode brakeMode_ = eBrakeMode::BRAKE):
            _pwmGenerator(pwmGenerator_),
            _ioDir(pinDir_),
            _ioNotEn(pinDis_),
            _enabled(false),
            _reversed(false),
            _goalCmd(0),
            _lastNonZeroCmd(false),
            _brakeMode(brakeMode_)
        {
            this->setEnabled(false);
            this->setReversed(reversed_);
            _ioDir.write(IO::eIOState::LOW_);
            _ioNotEn.write(IO::eIOState::LOW_);
        }

        void init(void)
        {
            _pwmGenerator.init();
        }

        void update(void)
        {
            float cmd = _goalCmd;
            bool isZeroCmd = IN_ERROR(cmd, ZERO_ERROR_TOLERANCE, 0.0F);

            // For the driver to brake, the dir pin must be switch when the pwm comes to 0% duty. Otherwise will coast
            if (isZeroCmd && _lastNonZeroCmd > 0.0F)
            {
                if (_brakeMode == eBrakeMode::BRAKE)
                {
                    cmd = -std::abs(ZERO_ERROR_TOLERANCE);
                }
                else
                {
                    cmd = std::abs(ZERO_ERROR_TOLERANCE);
                }
            }
            else if (isZeroCmd && _lastNonZeroCmd < 0.0F)
            {
                if (_brakeMode == eBrakeMode::BRAKE)
                {
                    cmd = std::abs(ZERO_ERROR_TOLERANCE);
                }
                else
                {
                    cmd = -std::abs(ZERO_ERROR_TOLERANCE);
                }
            }
            else if (isZeroCmd)  // On init
            {
                cmd = 0.0F;
            }

            bool isReversedCmd = (cmd < 0.0F);
            if (isReversedCmd)
            {
                _ioDir.write(IO::eIOState::LOW_);
            }
            else
            {
                _ioDir.write(IO::eIOState::HIGH_);
            }

            if (!isZeroCmd)
            {
                _lastNonZeroCmd = cmd;
            }

            LOG_INFO(Logger::Nodes::IFX9201SG,
                     "dir pin: %s, duty: %f",
                     (_ioDir.read() == IO::eIOState::HIGH_) ? "HIGH" : "LOW",
                     cmd);

            float duty = std::abs(cmd);
            _pwmGenerator.setDutyCycle(duty);
            _pwmGenerator.update();
        }

        void setCmd(float cmd_)
        {
            _goalCmd = CONSTRAIN(cmd_, -_maxCommand, _maxCommand);
            if (_reversed)
            {
                _goalCmd = -_goalCmd;
            }
        }

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

        void setEnabled(bool on_)
        {
            if (_enabled == on_)
            {
                return;
            }

            _enabled = on_;
            if (on_)
            {
                _ioNotEn.write(IO::eIOState::LOW_);
            }
            else
            {
                _ioNotEn.write(IO::eIOState::HIGH_);
            }
        }

        bool isEnabled(void) const
        {
            return _enabled;
        }

        void setReversed(bool reversed_)
        {
            _reversed = reversed_;
            this->setCmd(this->getCmd());  // Applies "reversed" on goal
        }

        bool isReversed(void) const
        {
            return _reversed;
        }

        void setBrakeMode(eBrakeMode mode_)
        {
            if (_brakeMode != mode_)
            {
                _brakeMode = mode_;
            }

            return;
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
        PWMGeneratorT& _pwmGenerator;

        IO::DigitalOutput _ioDir;
        IO::DigitalOutput _ioNotEn;

        float _maxCommand = MotorDrivers::MAX_CMD_OPEN_LOOP;

        bool _enabled;
        bool _reversed;
        float _goalCmd;

        float _lastNonZeroCmd;  // Needs to be tracked to control brake mode because driver has very weird brake/coast behavior

        eBrakeMode _brakeMode;

        VALIDATE_CONCEPT(MotorDriver, IFX9201SG<PWMGeneratorT>);
    };

    template<typename PwmGenerator_T>
    IFX9201SG(PwmGenerator_T&, gpio_num_t, bool, gpio_num_t, eBrakeMode) -> IFX9201SG<PwmGenerator_T>;

}  // namespace MotorDrivers

#endif  // ROVER_LIB2_MOTOR_DRIVERS_IFX9201SG_HPP
