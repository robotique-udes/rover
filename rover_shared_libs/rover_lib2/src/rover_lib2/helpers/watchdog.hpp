#ifndef ROVER_LIB2_HELPERS_WATCHDOG_HPP
#define ROVER_LIB2_HELPERS_WATCHDOG_HPP

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

    bool isOk(void) const
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

#endif  // ROVER_LIB2_HELPERS_WATCHDOG_HPP
