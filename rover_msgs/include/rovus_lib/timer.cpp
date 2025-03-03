#include "timer.hpp"

namespace RoverLib
{
#if defined(__linux__)
    uint64_t millis()
    {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        return (ts.tv_sec * 1'000) + (ts.tv_nsec / 1'000'000);
    }

    uint64_t micros()
    {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        return (ts.tv_sec * 1'000'000) + (ts.tv_nsec / 1'000);
    }

    uint64_t nanos()
    {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        return (ts.tv_sec * 1'000'000'000) + ts.tv_nsec;
    }
#endif  // defined(__linux__)

}  // namespace RoverLib
