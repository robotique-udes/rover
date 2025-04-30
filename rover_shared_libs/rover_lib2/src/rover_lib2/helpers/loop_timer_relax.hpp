#ifndef LOOP_TIMER_RELAX_HPP
#define LOOP_TIMER_RELAX_HPP

#include "loop_timer.hpp"

/**
 * @brief Same implementation as LoopTimer but LoopTimerRelax will not set the controller state in error if a loop is
 * missed
 *
 * @tparam ClockT Return type of the passed clock function -> ex: uint64_t
 * @tparam (*CLOCK_FUNC)(void) Function ptr to a clock function -> ex: Time::millis
 */
template<typename ClockT, ClockT (*CLOCK_FUNC)(void)>
class LoopTimerRelax : public LoopTimer<ClockT, CLOCK_FUNC>
{
  public:
    LoopTimerRelax(ClockT interval_):
        LoopTimer<ClockT, CLOCK_FUNC>(interval_)
    {
    }

    bool isReady() override
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
        }

        return true;
    }
};

#endif  // LOOP_TIMER_RELAX_HPP
