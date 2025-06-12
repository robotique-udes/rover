#ifndef ROVER_LIB2_HELPERS_CHRONO_HPP
#define ROVER_LIB2_HELPERS_CHRONO_HPP

/// @brief Chronometer helper class
/// @tparam TYPE The return type of the clock function used in 2nd template
/// argument
/// @tparam CLOCK_FUNC A function that returns the current clock of the
/// processor. In arduino framework: millis or micros
/// @example
/// void setup()
/// {
///     RoverHelpers::Chrono<unsigned long, micros> chrono;
///     delay(2000);
///     chrono.pause
///     Serial.printf("Time: %u", chrono.getTime());
///
///     Output: "Time: 200010"
/// }
template<typename ClockT, ClockT (*CLOCK_FUNC)(void)>
class Chrono
{
  public:
    Chrono(void)
    {
        _paused = false;
        _accumulatedTime = 0;
        _startClock = 0;
        this->init();
    }

    ~Chrono(void) {}

    void init(void)
    {
        _accumulatedTime = 0;
        _startClock = CLOCK_FUNC();
    }

    void pause(void)
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
            _startClock = CLOCK_FUNC();
            _paused = false;
        }
    }

    ClockT getTime(void)
    {
        if (_paused)
        {
            return _accumulatedTime;
        }
        else
        {
            return _accumulatedTime + CLOCK_FUNC() - _startClock;
        }
    }

    void restart(void)
    {
        this->start();
    }

    void start(void)
    {
        this->init();
    }

  private:
    ClockT _startClock;
    ClockT _accumulatedTime;
    bool _paused;
};

#endif  // ROVER_LIB2_HELPERS_CHRONO_HPP
