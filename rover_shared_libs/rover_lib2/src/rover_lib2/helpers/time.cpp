#include <ctime>
#include "time.hpp"

namespace Time
{
    uint64_t millis()
    {
#if defined(__linux__)
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        return (ts.tv_sec * 1'000) + (ts.tv_nsec / 1'000'000);

#elif defined(ARDUINO_ESP32S3_DEV)
        return static_cast<uint64_t>(esp_timer_get_time()) / 1000ULL;

#else
#error Not supported
#endif  // defined(__linux__)
    }

    uint64_t micros()
    {
#if defined(__linux__)
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        return (ts.tv_sec * 1'000'000) + static_cast<unsigned long>(ts.tv_nsec / 1'000L);

#elif defined(ARDUINO_ESP32S3_DEV)
        return static_cast<uint64_t>(esp_timer_get_time());

#else
#error Not supported
#endif  // defined(__linux__)
    }

    uint64_t nanos()
    {
#if defined(__linux__)
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        return (ts.tv_sec * 1'000'000'000) + ts.tv_nsec;

#elif defined(ARDUINO_ESP32S3_DEV)
        return static_cast<uint64_t>(esp_timer_get_time() * 1000LL);

#else
#error Not supported
#endif  // defined(__linux__)
    }

}  // namespace Time
