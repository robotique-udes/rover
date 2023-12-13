#ifndef __TIMER_H__
#define __TIMER_H__

#include "Arduino.h"

class TimerMillis
{
private:
    unsigned long _prevMillis;
    unsigned long _interval;

    void setInterval(unsigned long interval)
    {
        _interval = interval;
    }

public:
    TimerMillis()
    {
        _interval = 0;
        _prevMillis = 0;
    }
    TimerMillis(unsigned long interval)
    {
        _interval = 0;
        _prevMillis = 0;
        this->init(interval);
    }
    ~TimerMillis() {}

    void init(unsigned long interval)
    {
        setInterval(interval);
        _prevMillis = millis();
    }

    bool done(bool reset = 1)
    {
        if ((_prevMillis + _interval) < millis())
        {
            if (reset)
            {
                _prevMillis = millis() - 1;
            }
            return true;
        }
        else
        {
            return false;
        }
    }
};

#endif //__TIMER_H__
