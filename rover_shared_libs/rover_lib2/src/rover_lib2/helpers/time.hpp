#ifndef TIME_HPP
#define TIME_HPP

#if defined(__linux__)
#include <cstdint>
#elif defined(ARDUINO_ESP32S3_DEV)
#include "esp_timer.h"
#endif  // defined(__linux__)

namespace Time
{
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
