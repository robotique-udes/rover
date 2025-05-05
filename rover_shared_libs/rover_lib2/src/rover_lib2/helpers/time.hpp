#ifndef TIME_HPP
#define TIME_HPP

#if defined(__linux__)
#include <cstdint>
#include <thread>
#elif defined(ARDUINO_ESP32S3_DEV)
#include "esp_timer.h"
#endif  // defined(__linux__)

namespace Time
{
    // // TIME_TYPE ex: std::chrono::microseconds
    // template<typename TIME_TYPE>
    // class TimerFixedLoop
    // {
    //   public:
    //     TimerFixedLoop(TIME_TYPE interval_us_)
    //     {
    //         _interval = interval_us_;
    //         _nextLoopTime = std::chrono::steady_clock::now();
    //         _nextLoopTime += interval_us_;
    //     }

    //     void sleepUntilReady()
    //     {
    //         std::this_thread::sleep_until(_nextLoopTime);
    //         _nextLoopTime += _interval;
    //     }

    //   private:
    //     TIME_TYPE _interval;
    //     std::chrono::_V2::steady_clock::time_point _nextLoopTime;
    // };

    uint64_t millis(void);
    uint64_t micros(void);
    uint64_t nanos(void);
}  // namespace Time

// Necessary because of the hardcoded paths in platformio...
#if defined(ARDUINO_ESP32S3_DEV) || defined(TEST_NATIVE)
#ifndef TIME_CPP
// #include "time.cpp"
#endif  //  TIME_CPP
#endif  // defined(ARDUINO_ESP32S3_DEV) || defined(TEST_NATIVE)

#endif  // TIME_HPP
