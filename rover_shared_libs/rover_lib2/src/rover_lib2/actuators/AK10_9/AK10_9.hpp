#ifndef ROVER_LIB2_ACTUATORS_AK10_9_AK10_9_HPP
#define ROVER_LIB2_ACTUATORS_AK10_9_AK10_9_HPP

#include <concepts>
#include <Stream.h>

#include "AK10_9_internals.hpp"

#include "rover_lib2/helpers/log.hpp"
#include "rover_lib2/helpers/log_plot.hpp"

#include "rover_lib2/helpers/assert.hpp"
#include "rover_lib2/helpers/macros.hpp"
#include "rover_lib2/rover_object.hpp"
#include "rover_lib2/actuators/actuator.hpp"

#include "rover_lib2/sensors/encoder/encoder.hpp"
#include "rover_lib2/controllers/controller.h"
#include "rover_lib2/controllers/PID.hpp"
#include "rover_lib2/rover_object.hpp"

DEFINE_LOG_NODE(AK109, Logger::eNodeState::OFF);
DEFINE_LOG_NODE(AK109Plot, Logger::eNodeState::OFF);

namespace Actuators
{
    class AK10_9
    {
        static constexpr float RAD_S_TO_RPM = 9.549296596425384F;
        static constexpr float N_POLE_PAIRS = 17.0F;
        static constexpr float MOTOR_REDUCTION = 6.0F;
        static constexpr float FULL_STOP_CMD = 0.0F;

      public:
        enum class eControlType
        {
            SPEED,
            POSITION,
            TORQUE,
        };

      private:
        enum class eSendCmd
        {
            FRAME_HEAD = 0,
            DATA_LENGTH,
            COMMAND_SET_RPM,
            RPM_MSB,
            RPM_BYTE2,
            RPM_BYTE1,
            RPM_LSB,
            CHECKSUM_MSB,
            CHECKSUM_LSB,
            FRAME_TAIL,
            eLAST
        };

      public:
        AK10_9(eControlType controlType_, std::reference_wrapper<Stream> motorSerial_):
            _controlType(controlType_),
            _motorSerial(motorSerial_)
        {
        }

        void init() {}

        void update(void)
        {
            if (_errorMode)
            {
                LOG_WARN(Logger::Nodes::AK109, "Fallen in error mode, motor stopped");
                this->sendRadSCmd(FULL_STOP_CMD);
                return;
            }

            switch (_controlType)
            {
                case eControlType::SPEED:
                    this->speedModeUpdate();
                    break;
                case eControlType::POSITION:
                    [[fallthrough]];
                case eControlType::TORQUE:
                    [[fallthrough]];
                default:
                    ASSERT_MSG_ARGS("Control mode \"%u\" not supported, falling in safe mode", _controlType);
                    _errorMode = true;
            }
        }

        void speedModeUpdate(void)
        {
            float cmd = _goalSpeed;
            if (_maxJointLimit.has_value() && this->getPosition() >= _maxJointLimit.value())
            {
                cmd = std::clamp(cmd, -_maxSpeed, 0.0F);
            }

            if (_minJointLimit.has_value() && this->getPosition() <= _minJointLimit.value())
            {
                cmd = std::clamp(cmd, 0.0F, _maxSpeed);
            }

            LOG_DEBUG(Logger::Nodes::AK109, "_goalSpeed: %f | cmd: %f", _goalSpeed, cmd);
            LOG_PLOT(Logger::Nodes::AK109Plot, _goalSpeed, cmd, this->getSpeed(), this->getPosition());

            this->sendRadSCmd(cmd);
        }

        void setPosition(float /*goalPosition_*/)
        {
            ASSERT_MSG("Not implemented");
        }

        float getPosition(void) const
        {
            ASSERT_MSG("Not implemented");
            return 0.0F;
        }

        void setSpeed(float goalSpeedRad_)
        {
            _goalSpeed = goalSpeedRad_;
        }

        float getSpeed(void) const
        {
            ASSERT_MSG("Not implemented");
            return 0.0F;
        }

        void setMaxSpeed(float maxSpeed_)
        {
            _maxSpeed = maxSpeed_;
        }

        void calib(float /*offset_*/)
        {
            ASSERT_MSG("Not implemented");
        }

        void setJointLimit(std::optional<float> min_, std::optional<float> max_)
        {
            ASSERT_MSG("Not supported until get_position is implemented");
            if (min_.has_value() && max_.has_value() && min_.value() > max_.value())
            {
                ASSERT_MSG_ARGS("Invalid joint limit passed [%.3f; %.3f], actuator going into safe mode",
                                min_.value(),
                                max_.value());
                _errorMode = true;
            }

            _minJointLimit = min_;
            _maxJointLimit = max_;
        }

      private:
        void sendRadSCmd(float rad_s_)
        {
            float rpm = radSToRPM(rad_s_);
            float elecRpm_f = radToERPM(rpm);

            constexpr float MAX_ERPM = Actuators::AK10_9_Constants::RATED_SPEED_ERPM;
            elecRpm_f = std::clamp(elecRpm_f, -MAX_ERPM, MAX_ERPM);

            int32_t eRpm = static_cast<int32_t>(ROUND(elecRpm_f));

            _sendCmdBuffer[TO_UNDERLYING(eSendCmd::FRAME_HEAD)] = Actuators::AK10_9_Constants::FRAME_HEAD;
            _sendCmdBuffer[TO_UNDERLYING(eSendCmd::DATA_LENGTH)]
                = sizeof(eRpm) + sizeof(Actuators::AK10_9_Constants::COMMAND_SET_RPM);
            _sendCmdBuffer[TO_UNDERLYING(eSendCmd::COMMAND_SET_RPM)] = Actuators::AK10_9_Constants::COMMAND_SET_RPM;
            _sendCmdBuffer[TO_UNDERLYING(eSendCmd::RPM_MSB)] = (eRpm >> 24) & 0xFF;
            _sendCmdBuffer[TO_UNDERLYING(eSendCmd::RPM_BYTE2)] = (eRpm >> 16) & 0xFF;
            _sendCmdBuffer[TO_UNDERLYING(eSendCmd::RPM_BYTE1)] = (eRpm >> 8) & 0xFF;
            _sendCmdBuffer[TO_UNDERLYING(eSendCmd::RPM_LSB)] = eRpm & 0xFF;

            uint16_t checksum = this->calcCheckSum(_sendCmdBuffer.data() + 2, 5);

            _sendCmdBuffer[TO_UNDERLYING(eSendCmd::CHECKSUM_MSB)] = (checksum >> 8) & 0xFF;
            _sendCmdBuffer[TO_UNDERLYING(eSendCmd::CHECKSUM_LSB)] = checksum & 0xFF;
            _sendCmdBuffer[TO_UNDERLYING(eSendCmd::FRAME_TAIL)] = Actuators::AK10_9_Constants::FRAME_TAIL;

            _motorSerial.get().write(_sendCmdBuffer.data(), sizeof(_sendCmdBuffer));
        }

        uint16_t calcCheckSum(unsigned char* buf_, unsigned int len_) const
        {
            uint16_t cksum = 0;
            for (size_t i = 0; i < len_; i++)
            {
                cksum = Actuators::AK10_9_Constants::CRC16_TAB[(((cksum >> 8) ^ *buf_++) & 0xFF)] ^ (cksum << 8);
            }
            return cksum;
        }

        float radSToRPM(float radPerSec_) const
        {
            return radPerSec_ * RAD_S_TO_RPM;
        }

        float radToERPM(float radPerSec_) const
        {
            return radPerSec_ * RAD_S_TO_RPM * N_POLE_PAIRS * MOTOR_REDUCTION;
        }

        const eControlType _controlType;
        bool _errorMode = false;

        std::array<uint8_t, TO_UNDERLYING(eSendCmd::eLAST)> _sendCmdBuffer = {0};

        std::reference_wrapper<Stream> _motorSerial;

        float _goalPos = 0.0F;
        std::optional<float> _minJointLimit = std::nullopt;
        std::optional<float> _maxJointLimit = std::nullopt;

        float _goalSpeed = 0.0F;
        float _maxSpeed = std::numeric_limits<float>::max();

        VALIDATE_CONCEPT(Actuator, AK10_9);
    };
}  // namespace Actuators

#endif  // ROVER_LIB2_ACTUATORS_AK10_9_AK10_9_HPP
