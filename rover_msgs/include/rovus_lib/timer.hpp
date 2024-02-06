#ifndef __TIMER_HPP__
#define __TIMER_HPP__

#include <chrono>

namespace RovusLib
{
    unsigned long millis()
    {
        using namespace std::chrono;
        return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
    }

    unsigned long micros()
    {
        using namespace std::chrono;
        return duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count();
    }

    template <typename T, T (*clockFunc)(void)>
    class Timer
    {
    public:
        Timer()
        {
            _interval = static_cast<T>(0);
            _prevClock = clockFunc();
        }
        Timer(T interval)
        {
            _interval = static_cast<T>(0);
            _prevClock = clockFunc();
            this->init(interval);
        }
        ~Timer() {}

        void init(T interval)
        {
            setInterval(interval);
            _prevClock = clockFunc();
        }

        void reset()
        {
            _prevClock = clockFunc();
        }

        bool isDone(bool reset = 1)
        {
            if ((_prevClock + _interval) < clockFunc())
            {
                if (reset)
                {
                    _prevClock = clockFunc();
                }
                return true;
            }
            else
            {
                return false;
            }
        }

    private:
        T _prevClock;
        T _interval;

        void setInterval(T interval)
        {
            _interval = interval;
        }
    };

    template <typename T, T (*clockFunc)(void)>
    class Chrono
    {
    public:
        Chrono()
        {
            this->start();
        }
        ~Chrono() {}

        void start()
        {
            startTime = clockFunc();
        }

        T getTime()
        {
            return clockFunc() - startTime;
        }

    private:
        T startTime;
    };
}

#endif // __TIMER_HPP__
