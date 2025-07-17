#ifndef ROVER_LIB2_CONTROLLERS_PID_HPP
#define ROVER_LIB2_CONTROLLERS_PID_HPP

#include "rover_lib2/helpers/time.hpp"
#include "rover_lib2/helpers/macros.hpp"

#include <cmath>

namespace Controllers
{
    static constexpr uint64_t PERIOD_CALC_US = 10'000ULL;
    static constexpr float DEFAULT_ERROR_TOLERANCE = 0.001F;

    // concept Controller
    class PID
    {
      public:
        PID(float kp_,
            float ki_,
            float kd_,
            float integralLimit_,
            uint64_t calcPeriod_ = PERIOD_CALC_US,
            float errorTolerance_ = DEFAULT_ERROR_TOLERANCE):
            _calcPeriod(calcPeriod_),
            _kp(kp_),
            _ki(ki_),
            _kd(kd_),
            _integralLimit(std::abs(integralLimit_)),
            _errorTolerance(std::abs(errorTolerance_))
        {
            _lastMeasureTime = Time::micros();
            this->reset();
        }

        float computeCommand(float input_, float target_)
        {
            uint64_t currentTime = Time::micros();
            if (currentTime - _lastMeasureTime < _calcPeriod)
            {
                return _lastcmd;
            }

            if (IN_ERROR(input_, _errorTolerance, target_))
            {
                return 0.0F;
            }

            uint64_t dt = currentTime - _lastMeasureTime;
            _lastMeasureTime = currentTime;

            float error = target_ - input_;
            if (std::isnan(error))
            {
                error = 0.0f;
            }
            if (std::isinf(error))
            {
                error = std::numeric_limits<float>::max();
            }

            float cmdD = 0.0f;
            float dtSec = static_cast<float>(dt) / 1'000'000.0f;
            if (dtSec != 0.0F)
            {
                // TODO: Test and consider using input instead of error to prevent D kick on target changes
                cmdD = _kd * (error - _previousError) / dtSec;
            }
            else
            {
                cmdD = 0.0F;
            }

            _cmdI += _ki * error;
            _cmdI = std::clamp(_cmdI, -_integralLimit, _integralLimit);

            float cmdP = _kp * error;

            if (std::isnan(cmdP))
            {
                cmdP = 0.0f;
            }
            if (std::isnan(_cmdI))
            {
                _cmdI = 0.0f;
            }
            if (std::isnan(cmdD))
            {
                cmdD = 0.0f;
            }

            _previousError = error;

            _lastcmd = cmdP + _cmdI + cmdD;
            return _lastcmd;
        }

        void reset()
        {
            _cmdI = 0.0F;
            _previousError = 0.0F;
            _lastcmd = 0.0F;
            _lastMeasureTime = Time::micros();
        }

        void setGains(float kp_, float ki_, float kd_)
        {
            _kp = kp_;
            _ki = ki_;
            _kd = kd_;
        }

      private:
        const uint64_t _calcPeriod;

        float _kp;
        float _ki;
        float _kd;
        const float _integralLimit;

        const float _errorTolerance;

        float _lastcmd = 0.0f;
        float _cmdI = 0.0f;
        float _previousError = 0.0f;
        uint64_t _lastMeasureTime = Time::micros();
    };

}  // namespace Controllers

#endif  // ROVER_LIB2_CONTROLLERS_PID_HPP
