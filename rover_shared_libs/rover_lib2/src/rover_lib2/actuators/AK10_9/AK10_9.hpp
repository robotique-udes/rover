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
DEFINE_LOG_NODE(AK109Plot, Logger::eNodeState::ON);

namespace Actuators
{
    static constexpr float RAD_S_TO_RPM = 9.549296596425384F;
    static constexpr float N_POLE_PAIRS = 17.0F;
    static constexpr float MOTOR_REDUCTION = 6.0F;
    static constexpr float FULL_STOP_SPEED = 0.0F;

    enum class eControlType
    {
        SPEED,
        POSITION,
        TORQUE,
    };

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

    template<Encoders::Encoder EncoderT = Encoders::None,
             Controllers::Controller PositionControllerT = Controllers::None,
             Controllers::Controller SpeedControllerT = Controllers::None>
    class AK109
    {
      public:
        AK109(eControlType controlType_,
              Stream* motorSerial_,
              EncoderT* encoder_ = nullptr,
              PositionControllerT* controllerPos_ = nullptr,
              SpeedControllerT* controllerSpeed_ = nullptr):
            _controlType(controlType_),
            _motorSerial(motorSerial_),
            _pEncoder(encoder_),
            _pPositionController(controllerPos_),
            _pSpeedController(controllerSpeed_)
        {
        }

        void init()
        {
            if (_pEncoder)
            {
                _pEncoder->init();
            }
            if (_pPositionController)
            {
                _pPositionController->reset();
            }
            if (_pSpeedController)
            {
                _pSpeedController->reset();
            }
        }

        void update(void)
        {
            if (_pEncoder)
            {
                _pEncoder->update();
            }

            if (_errorMode)
            {
                LOG_WARN(Logger::Nodes::AK109, "Fallen in error mode, motor stopped");
                this->setSpeed(FULL_STOP_SPEED);
                return;
            }

            switch (_controlType)
            {
                case eControlType::SPEED:
                    if (_pSpeedController)
                    {
                        this->speedModeUpdate();
                    }
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
            if (!_pSpeedController || !_pEncoder)
            {
                LOG_ERROR(Logger::Nodes::AK109, "No speed controller or encoder attached, falling in error mode");
                _errorMode = true;
                return;
            }

            float cmd = _pSpeedController->computeCommand(this->getSpeed(), _goalSpeed);

            if (_maxJointLimit.has_value() && this->getPosition() >= _maxJointLimit.value())
            {
                cmd = std::clamp(cmd, std::numeric_limits<float>::lowest(), 0.0F);
            }

            if (_minJointLimit.has_value() && this->getPosition() <= _minJointLimit.value())
            {
                cmd = std::clamp(cmd, 0.0F, std::numeric_limits<float>::max());
            }

            LOG_DEBUG(Logger::Nodes::AK109, "_goalSpeed: %f | cmd: %f", _goalSpeed, cmd);

            LOG_PLOT(Logger::Nodes::AK109Plot, _goalSpeed, cmd, this->getSpeed(), this->getPosition());

            this->sendCmd(cmd);
        }

        void setPosition(float goalPosition_)
        {
            ASSERT_MSG("AK109 does not support position control");
        }

        float getPosition(void) const
        {
            if (!_pEncoder)
            {
                LOG_DEBUG(Logger::Nodes::AK109, "No encoder attached to actuator, returning 0.0F");
                return 0.0F;
            }

            return _pEncoder->getPosition();
        }

        void setSpeed(float goalSpeedRad_)
        {
            _goalSpeed = goalSpeedRad_;
        }

        void sendCmd(float cmd_)
        {
            float rpm = toRPM(cmd_);
            float elecRpm_f = toelectricRPM(rpm);

            constexpr float MAX_ERPM = AK10_9::RATED_SPEED_ERPM;
            elecRpm_f = std::clamp(elecRpm_f, -MAX_ERPM, MAX_ERPM);

            int32_t eRpm = static_cast<int32_t>(elecRpm_f);

            _sendCmdBuffer[TO_UNDERLYING(eSendCmd::FRAME_HEAD)] = AK10_9::FRAME_HEAD;
            _sendCmdBuffer[TO_UNDERLYING(eSendCmd::DATA_LENGTH)] = 0x05;
            _sendCmdBuffer[TO_UNDERLYING(eSendCmd::COMMAND_SET_RPM)] = AK10_9::COMMAND_SET_RPM;
            _sendCmdBuffer[TO_UNDERLYING(eSendCmd::RPM_MSB)] = (eRpm >> 24) & 0xFF;
            _sendCmdBuffer[TO_UNDERLYING(eSendCmd::RPM_BYTE2)] = (eRpm >> 16) & 0xFF;
            _sendCmdBuffer[TO_UNDERLYING(eSendCmd::RPM_BYTE1)] = (eRpm >> 8) & 0xFF;
            _sendCmdBuffer[TO_UNDERLYING(eSendCmd::RPM_LSB)] = eRpm & 0xFF;

            uint16_t checksum = this->calcCheckSum(_sendCmdBuffer.data() + 2, 5);

            _sendCmdBuffer[TO_UNDERLYING(eSendCmd::CHECKSUM_MSB)] = (checksum >> 8) & 0xFF;
            _sendCmdBuffer[TO_UNDERLYING(eSendCmd::CHECKSUM_LSB)] = checksum & 0xFF;
            _sendCmdBuffer[TO_UNDERLYING(eSendCmd::FRAME_TAIL)] = AK10_9::FRAME_TAIL;

            _motorSerial->write(_sendCmdBuffer.data(), sizeof(_sendCmdBuffer));
        }

        float getSpeed(void) const
        {
            if (!_pEncoder)
            {
                LOG_DEBUG(Logger::Nodes::AK109, "No encoder attached to actuator, returning 0.0F");
                return 0.0F;
            }

            return _pEncoder->getSpeed();
        }

        void setMaxSpeed(float maxSpeed_)
        {
            _maxSpeed = maxSpeed_;
        }

        void calib(float offset_)
        {
            if (_pEncoder)
            {
                _pEncoder->calib(offset_);
            }
        }

        void setJointLimit(std::optional<float> min_, std::optional<float> max_)
        {
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

        uint16_t calcCheckSum(unsigned char* buf_, unsigned int len_) const
        {
            uint16_t cksum = 0;
            for (size_t i = 0; i < len_; i++)
            {
                cksum = AK10_9::CRC16_TAB[(((cksum >> 8) ^ *buf_++) & 0xFF)] ^ (cksum << 8);
            }
            return cksum;
        }

        float toRPM(float radPerSec_) const
        {
            return radPerSec_ * RAD_S_TO_RPM;
        }

        float toelectricRPM(float radPerSec_) const
        {
            return radPerSec_ * RAD_S_TO_RPM * N_POLE_PAIRS * MOTOR_REDUCTION;
        }

      private:
        const eControlType _controlType;

        bool _errorMode = false;

        std::array<uint8_t, TO_UNDERLYING(eSendCmd::eLAST)> _sendCmdBuffer = {0};

        Stream* _motorSerial = nullptr;
        EncoderT* _pEncoder = nullptr;
        PositionControllerT* _pPositionController = nullptr;
        SpeedControllerT* _pSpeedController = nullptr;

        float _goalPos = 0.0F;
        std::optional<float> _minJointLimit = std::nullopt;
        std::optional<float> _maxJointLimit = std::nullopt;

        float _goalSpeed = 0.0F;
        float _maxSpeed = std::numeric_limits<float>::max();

        VALIDATE_CONCEPT(Actuator, AK109);
    };
}  // namespace Actuators

#endif  // ROVER_LIB2_ACTUATORS_AK10_9_AK10_9_HPP
