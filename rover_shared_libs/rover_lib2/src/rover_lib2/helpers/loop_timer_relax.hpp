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
        if (currentTime < this->_nextTriggerTime)
        {
            return false;
        }

        this->_nextTriggerTime += this->_interval;
        if (currentTime >= this->_nextTriggerTime)
        {
            if (currentTime >= this->_nextTriggerTime + this->_interval)
            {
                this->_nextTriggerTime = currentTime + this->_interval;
            }
            else
            {
                this->_nextTriggerTime += this->_interval;
            }
        }

        return true;
    }
};

#endif  // LOOP_TIMER_RELAX_HPP
