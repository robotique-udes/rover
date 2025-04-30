#ifndef ONE_SHOT_TIMER_HPP
#define ONE_SHOT_TIMER_HPP

#include "rover_lib2/helpers/chrono.hpp"

template<typename ClockT, ClockT (*CLOCK_FUNC)(void)>
class OneShotTimer
{
  public:
    OneShotTimer(ClockT interval_):
        _interval(interval_),
        _triggered(false)
    {
    }

    bool isReady(void)
    {
        if (!_triggered && (chrono_.getTime() > _interval))
        {
            _triggered = true;
            return true;
        }
        else
        {
            return false;
        }
    }

  private:
    ClockT _interval;
    Chrono<ClockT, CLOCK_FUNC> chrono_;

    bool _triggered;
};

#endif  // ONE_SHOT_TIMER_HPP
