#ifndef __TIMER_HPP__
#define __TIMER_HPP__

#if not defined(__linux__)
#error OS Not supported
#endif  // not defined(__linux__)

#if defined(__linux__)
#include <cstdint>
#include <thread>
#endif  // defined(__linux__)

namespace RoverLib
{
    // #if defined(__linux__)
    template<typename TYPE, TYPE (*clockFunc)(void)>
    class Timer
    {
      private:
        TYPE _prevClock;
        TYPE _interval;

        void setInterval(TYPE interval)
        {
            _interval = interval;
        }

      public:
        Timer()
        {
            _interval = 0;
            _prevClock = 0;
        }
        Timer(TYPE interval_)
        {
            _interval = 0;
            _prevClock = 0;

            this->init(interval_);
        }
        ~Timer() {}

        void init(TYPE interval)
        {
            setInterval(interval);
            _prevClock = clockFunc();
        }

        bool isDone(bool reset = 1)
        {
            if ((_prevClock + _interval) < clockFunc())
            {
                if (reset)
                {
                    _prevClock = clockFunc() - 1;
                }
                return true;
            }
            else
            {
                return false;
            }
        }

        void updateInterval(TYPE newInterval_)
        {
            _interval = newInterval_;
        }

        void reset(void)
        {
            _prevClock = clockFunc();
        }
    };

    template<typename TYPE, TYPE (*clockFunc)(void)>
    class Chrono
    {
      private:
        TYPE _startClock;
        TYPE _accumulatedTime;
        bool _paused;

      public:
        Chrono()
        {
            _paused = false;
            _accumulatedTime = 0;
            _startClock = 0;
            this->init();
        }
        ~Chrono() {}

        void init()
        {
            _accumulatedTime = 0;
            _startClock = clockFunc();
        }

        TYPE pause(void)
        {
            if (!_paused)
            {
                _accumulatedTime = this->getTime();
                _paused = true;
            }
        }

        void resume(void)
        {
            if (_paused)
            {
                _startClock = clockFunc();
                _paused = false;
            }
        }

        TYPE getTime(void)
        {
            if (_paused)
            {
                return _accumulatedTime;
            }
            else
            {
                return _accumulatedTime + clockFunc() - _startClock;
            }
        }

        void restart(void)
        {
            this->init();
        }
    };

    // TIME_TYPE ex: std::chrono::microseconds
    template<typename TIME_TYPE>
    class TimerFixedLoop
    {
      public:
        TimerFixedLoop(TIME_TYPE interval_us_)
        {
            _interval = interval_us_;
            _nextLoopTime = std::chrono::steady_clock::now();
            _nextLoopTime += interval_us_;
        }

        void sleepUntilReady()
        {
            std::this_thread::sleep_until(_nextLoopTime);
            _nextLoopTime += _interval;
        }

      private:
        TIME_TYPE _interval;
        std::chrono::_V2::steady_clock::time_point _nextLoopTime;
    };

#if defined(__linux__)
    uint64_t millis();
    uint64_t micros();
    uint64_t nanos();
#endif  // defined(__linux__)

}  // namespace RoverLib

#endif  //__TIMER_HPP__
