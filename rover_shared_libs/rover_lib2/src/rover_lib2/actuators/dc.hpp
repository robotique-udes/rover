#ifndef ROVER_LIB2_ACTUATORS_DC_HPP
#define ROVER_LIB2_ACTUATORS_DC_HPP

#include "actuator.hpp"

#include "rover_lib2/helpers/macros.hpp"
#include "rover_lib2/helpers/assert.hpp"
#include "rover_lib2/helpers/log.hpp"
#include "rover_lib2/helpers/log_plot.hpp"

#include "rover_lib2/motor_drivers/motor_driver.hpp"
#include "rover_lib2/sensors/encoder/encoder.hpp"
#include "rover_lib2/controllers/controller.h"
#include "rover_lib2/controllers/PID.hpp"

DEFINE_LOG_NODE(ActuatorDc, Logger::eNodeState::OFF);
DEFINE_LOG_NODE(ActuatorDcPlot, Logger::eNodeState::OFF);

namespace Actuators
{
    enum class eControlType
    {
        SPEED,
        POSITION,
        TORQUE,
    };

    enum class eFeedbackType
    {
        OPEN_LOOP,
        CLOSE_LOOP,
    };

    /**
     * @brief
     * @warning Missing features: Position control, torque control, max_speed limiting, joint limit enforcements
     *
     */
    template<class DriverT,
             Encoders::Encoder EncoderT = Encoders::None,
             Controllers::Controller PositionControllerT = Controllers::None,
             Controllers::Controller SpeedControllerT = Controllers::None>
    class DC
    {
      public:
        DC(eControlType controlType_,
           eFeedbackType feedbackType_,
           DriverT& driver_,
           EncoderT* encoder_ = nullptr,
           PositionControllerT* controllerPos_ = nullptr,
           SpeedControllerT* controllerSpeed_ = nullptr):
            _controlType(controlType_),
            _feedbackType(feedbackType_),
            _motorDriver(driver_),
            _pEncoder(encoder_),
            _pControllerPos(controllerPos_),
            _pControllerSpeed(controllerSpeed_)
        {
            switch (_controlType)
            {
                case eControlType::SPEED:
                    switch (_feedbackType)
                    {
                        case eFeedbackType::OPEN_LOOP:
                            break;
                        case eFeedbackType::CLOSE_LOOP:
                            ASSERT_COND_MSG(_pControllerSpeed, "Can't control in speed without valid speed controller");
                            ASSERT_COND_MSG(_pEncoder, "Can't control in speed without valid encoder");
                            break;
                        default:
                            ASSERT_MSG_ARGS("Unknown control mode specified: %u", std::to_underlying(_feedbackType));
                            _errorMode = true;
                            break;
                    }
                    break;

                case eControlType::POSITION:
                    ASSERT_COND_MSG(_feedbackType == eFeedbackType::CLOSE_LOOP,
                                    "Can't control DC motor in position in open loop");
                    ASSERT_COND_MSG(_pEncoder, "Can't control in position without valid encoder");
                    ASSERT_COND_MSG(_pControllerPos, "Can't control in position without valid position controller");
                    [[fallthrough]];

                case eControlType::TORQUE:
                    [[fallthrough]];

                default:
                    ASSERT_MSG_ARGS("Specified control mode not supported: %u", std::to_underlying(_controlType));
                    _errorMode = true;
            }
        }

        void init()
        {
            _motorDriver.init();

            if (_pEncoder)
            {
                _pEncoder->init();
            }

            if (_pControllerPos)
            {
                _pControllerPos->reset();
            }

            if (_pControllerSpeed)
            {
                _pControllerSpeed->reset();
            }
        }

        void update()
        {
            _motorDriver.update();

            if (_pEncoder)
            {
                _pEncoder->update();
            }

            if (_errorMode)
            {
                LOG_WARN(Logger::Nodes::ActuatorDc, "Fallen in error mode, motor stopped");
                _motorDriver.setCmd(0.0F);
                return;
            }

            switch (_controlType)
            {
                case eControlType::SPEED:
                    speedModeUpdate();
                    break;
                case eControlType::POSITION:
                    [[fallthrough]];
                case eControlType::TORQUE:
                    [[fallthrough]];
                default:
                    ASSERT_MSG_ARGS("Control mode \"%u\" not supported, falling in safe mode", _controlType);
                    _errorMode = true;
                    break;
            }
        }

        void setPosition(float pos_)
        {
            _goalPos = pos_;
        }

        float getPosition() const
        {
            if (!_pEncoder)
            {
                LOG_DEBUG(Logger::Nodes::ActuatorDc, "No encoder attached to actuator, returning 0.0F");
                return 0.0F;
            }

            return _pEncoder->getPosition();
        }

        void setSpeed(float speed_)
        {
            _goalSpeed = speed_;
        }

        float getSpeed() const
        {
            if (!_pEncoder)
            {
                LOG_DEBUG(Logger::Nodes::ActuatorDc, "No encoder attached to actuator, returning 0.0F");
                return 0.0F;
            }

            return _pEncoder->getSpeed();
        }

        void setMaxSpeed(float max_speed_)
        {
            _maxSpeed = max_speed_;
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

        void calib(float offset_)
        {
            if (_pEncoder)
            {
                _pEncoder->calib(offset_);
            }
        }

      private:
        void speedModeUpdate(void)
        {
            LOG_DEBUG(Logger::Nodes::ActuatorDc, "In Speed Mode update");
            switch (_feedbackType)
            {
                case eFeedbackType::OPEN_LOOP:
                    LOG_DEBUG(Logger::Nodes::ActuatorDc, "In open loop mode");
                    _motorDriver.setCmd(_goalSpeed);
                    break;

                case eFeedbackType::CLOSE_LOOP:
                {
                    LOG_DEBUG(Logger::Nodes::ActuatorDc, "In closed loop mode");
                    if (!_pControllerSpeed || !_pEncoder)
                    {
                        _errorMode = true;
                        return;
                    }

                    float cmd = _pControllerSpeed->computeCommand(this->getSpeed(), _goalSpeed);

                    if (_maxJointLimit.has_value() && this->getPosition() >= _maxJointLimit.value())
                    {
                        cmd = std::clamp(cmd, std::numeric_limits<float>::lowest(), 0.0F);
                    }

                    if (_minJointLimit.has_value() && this->getPosition() <= _minJointLimit.value())
                    {
                        cmd = std::clamp(cmd, 0.0F, std::numeric_limits<float>::max());
                    }

                    _motorDriver.setCmd(cmd);
                    LOG_INFO(Logger::Nodes::ActuatorDc,
                             "_goalSpeed: %f, this->getSpeed(): %f | cmd: %f",
                             _goalSpeed,
                             this->getSpeed(),
                             cmd);

                    LOG_PLOT(Logger::Nodes::ActuatorDcPlot, _goalSpeed, this->getSpeed());
                    break;
                }

                default:
                    ASSERT_MSG("Shouldn't fall here, implementation error");
                    _errorMode = true;
            }
        }

        const eControlType _controlType;
        const eFeedbackType _feedbackType;

        bool _errorMode = false;

        DriverT& _motorDriver;
        EncoderT* _pEncoder;
        PositionControllerT* _pControllerPos;
        SpeedControllerT* _pControllerSpeed;

        float _goalPos = 0.0F;
        std::optional<float> _minJointLimit = std::nullopt;
        std::optional<float> _maxJointLimit = std::nullopt;

        float _goalSpeed = 0.0F;
        float _maxSpeed = std::numeric_limits<float>::max();

        VALIDATE_CONCEPT(Actuator, DC);
    };

    // ===========================================================================================================================
    // Template deduction guides
    // ===========================================================================================================================

    // Generic
    template<class DriverT, class EncoderT, class PositionControllerT, class SpeedControllerT>
    DC(eControlType, eFeedbackType, DriverT&, EncoderT*, PositionControllerT*, SpeedControllerT*)
        -> DC<DriverT,
              std::remove_pointer_t<EncoderT>,
              std::remove_pointer_t<PositionControllerT>,
              std::remove_pointer_t<SpeedControllerT>>;

    // Driver
    template<class DriverT>
    DC(eControlType, eFeedbackType, DriverT&, std::nullptr_t, std::nullptr_t, std::nullptr_t) -> DC<DriverT>;

    // Driver + encoder
    template<class DriverT, class EncoderT>
    DC(eControlType, eFeedbackType, DriverT&, EncoderT*, std::nullptr_t, std::nullptr_t)
        -> DC<DriverT, std::remove_pointer_t<EncoderT>>;

    // Driver + position controller
    template<class DriverT, class PositionControllerT>
    DC(eControlType, eFeedbackType, DriverT&, std::nullptr_t, PositionControllerT*, std::nullptr_t)
        -> DC<DriverT, Encoders::None, std::remove_pointer_t<PositionControllerT>>;

    // Driver + speed controller
    template<class DriverT, class SpeedControllerT>
    DC(eControlType, eFeedbackType, DriverT&, std::nullptr_t, std::nullptr_t, SpeedControllerT*)
        -> DC<DriverT, Encoders::None, Controllers::None, std::remove_pointer_t<SpeedControllerT>>;

    // Driver + encoder + position controller
    template<class DriverT, class EncoderT, class PositionControllerT>
    DC(eControlType, eFeedbackType, DriverT&, EncoderT*, PositionControllerT*, std::nullptr_t)
        -> DC<DriverT, std::remove_pointer_t<EncoderT>, std::remove_pointer_t<PositionControllerT>>;

    // Driver + encoder + speed controller
    template<class DriverT, class EncoderT, class SpeedControllerT>
    DC(eControlType, eFeedbackType, DriverT&, EncoderT*, std::nullptr_t, SpeedControllerT*)
        -> DC<DriverT, std::remove_pointer_t<EncoderT>, Controllers::None, std::remove_pointer_t<SpeedControllerT>>;
}  // namespace Actuators

#endif  // ROVER_LIB2_ACTUATORS_DC_HPP
