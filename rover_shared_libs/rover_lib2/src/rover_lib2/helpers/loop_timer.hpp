#ifndef LOOP_TIMER
#define LOOP_TIMER

#include "rover_lib2/helpers/chrono.hpp"
#include "rover_lib2/helpers/log.hpp"
#include "rover_lib2/helpers/time.hpp"

DEFINE_LOG_NODE(LoopTimer, Logger::eNodeState::OFF)

/**
 * @brief Object to help execute code at a specific rate/period. This object will set the controller HealthState in error if a
 * loop is missed to make sure actuators control loops or other timing critical components are "safe".
 *
 * @tparam ClockT Return type of the passed clock function -> ex: uint64_t
 * @tparam (*CLOCK_FUNC)(void) Function ptr to a clock function -> ex: Time::millis
 */
template<typename ClockT, ClockT (*CLOCK_FUNC)(void)>
class LoopTimer
{
  public:
    LoopTimer(ClockT interval_):
        _interval(interval_),
        _nextTriggerTime(CLOCK_FUNC()),
        _overrunFalsePositiveLatch(false)
    {
    }

    bool isReady()
    {
        const ClockT currentTime = CLOCK_FUNC();
        if (currentTime < _nextTriggerTime)
        {
            return false;
        }

        _nextTriggerTime += _interval;
        if (currentTime >= _nextTriggerTime)
        {
            if (currentTime >= _nextTriggerTime + _interval)
            {
                _nextTriggerTime = currentTime + _interval;
            }
            else
            {
                _nextTriggerTime += _interval;
            }

            if (_overrunFalsePositiveLatch)
            {
                LOG_ERROR(Logger::Nodes::LoopTimer, "Warning code execution too slow for specified interval");
                _overrunFalsePositiveLatch = true;
            }
        }

        return true;
    }

    void setInterval(ClockT interval_)
    {
        _interval = interval_;
        _nextTriggerTime = CLOCK_FUNC() + _interval;
    }

    void reset()
    {
        _nextTriggerTime = CLOCK_FUNC() + _interval;
    }

    ClockT getInterval() const
    {
        return _interval;
    }

  private:
    ClockT _interval;
    ClockT _nextTriggerTime;
    bool _overrunFalsePositiveLatch;
};

#endif  // LOOP_TIMER
