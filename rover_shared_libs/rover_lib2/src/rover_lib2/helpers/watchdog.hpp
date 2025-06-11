#ifndef WATCHDOG_HPP
#define WATCHDOG_HPP

#include "chrono.hpp"

template<typename ClockFuncReturnT, ClockFuncReturnT (*ClockFunc)(void)>
class Watchdog
{
  public:
    Watchdog(ClockFuncReturnT interval_):
        _interval(interval_)
    {
        this->reset();
    }

    bool isOk(void)
    {
        return (chrono.getTime() < _interval);
    }

    void reset(void)
    {
        chrono.restart();
    }

  private:
    Chrono<ClockFuncReturnT, ClockFunc> chrono;
    const ClockFuncReturnT _interval;
};

#endif  // WATCHDOG_HPP
