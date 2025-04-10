#ifndef LOOP_TIMER
#define LOOP_TIMER

#include "rover_lib2/helpers/chrono.hpp"
#include "rover_lib2/helpers/log.hpp"

DEFINE_LOG_NODE(LoopTimer, Logger::eNodeState::OFF)

template<typename ClockT, ClockT (*CLOCK_FUNC)(void)>
class LoopTimer
{
  public:
    LoopTimer(ClockT interval_)
    {
        this->setInterval(interval_);
    }

    bool isReady(void)
    {
        ClockT currentTime = CLOCK_FUNC();
        if (_nextTriggerTime > currentTime)
        {
            LOG_WARN(Logger::Nodes::LoopTimer, "Timer overflow detected, trying to handle");
            this->reset();
        }

        _nextTriggerTime = currentTime;

        uint64_t expectedTriggerCtn = static_cast<uint64_t>(currentTime / _interval);
        if (_triggerCtn < expectedTriggerCtn)
        {
            if (_triggerCtn != 0 && (expectedTriggerCtn - _triggerCtn) != 1ULL)
            {
                LOG_WARN(Logger::Nodes::LoopTimer, "Warning code execution too slow for specified interval");
            }

            _triggerCtn = expectedTriggerCtn;
            return true;
        }
        else
        {
            return false;
        }
    }

    void setInterval(ClockT newInterval_)
    {
        _nextTriggerTime = static_cast<ClockT>(0);
        _interval = newInterval_;
        _triggerCtn = 0ULL;
    }

    ClockT getInterval(void) const
    {
        return _interval;
    }

    void reset(void)
    {
        this->setInterval(this->getInterval());
    }

  private:
    ClockT _interval;
    uint64_t _triggerCtn;
    ClockT _nextTriggerTime;
};

#endif  // LOOP_TIMER
