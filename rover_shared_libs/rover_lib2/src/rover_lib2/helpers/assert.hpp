#ifndef ASSERT_HPP
#define ASSERT_HPP
#if defined(ARDUINO_ESP32S3_DEV)
#include "rover_lib2/helpers/log.hpp"
#include "driver/ledc.h"
#include "soc/mcpwm_periph.h"
#include "esp_system.h"
#include "esp_private/periph_ctrl.h"
#include <cstdarg>
#elif defined(__linux__)
#include <cstdarg>
#include <iostream>
#endif  // defined(ARDUINO_ESP32S3_DEV)

#if defined(ARDUINO_ESP32S3_DEV)
DEFINE_LOG_NODE(ASSERTS, Logger::eNodeState::ON);
namespace
{
    constexpr void SHUTDOWN_PWM(void)
    {
        for (int speed_mode = 0; speed_mode < (int)LEDC_SPEED_MODE_MAX; speed_mode++)
        {
            for (int channel = 0; channel < (int)LEDC_CHANNEL_MAX; channel++)
            {
                ledc_stop((ledc_mode_t)speed_mode, (ledc_channel_t)channel, 0);
            }
        }

        for (int i = 0; i < SOC_MCPWM_GROUPS; i++)
        {
            periph_module_disable(mcpwm_periph_signals.groups[i].module);
            periph_module_reset(mcpwm_periph_signals.groups[i].module);
        }
    }

    inline void ABORT(void)
    {
        SHUTDOWN_PWM();
#if defined(DEBUG)
        LOG_FLUSH();
        abort();

#endif  // defined(DEBUG)
    }
}  // namespace

// Implementation macros for ESP32
#define ASSERT()                                                 \
    do                                                           \
    {                                                            \
        LOG_FATAL(Logger::Nodes::ASSERTS, "Explicit assertion"); \
        ABORT();                                                 \
    }                                                            \
    while (0)

#define ASSERT_MSG(msg)                                                  \
    do                                                                   \
    {                                                                    \
        LOG_FATAL(Logger::Nodes::ASSERTS, "Explicit assertion:\n", msg); \
        ABORT();                                                         \
    }                                                                    \
    while (0)

#define ASSERT_MSG_ARGS(format, ...)                                                      \
    do                                                                                    \
    {                                                                                     \
        LOG_FATAL(Logger::Nodes::ASSERTS, "Explicit assertion:\n" format, ##__VA_ARGS__); \
        ABORT();                                                                          \
    }                                                                                     \
    while (0)

#define ASSERT_COND(condition)                                                     \
    do                                                                             \
    {                                                                              \
        if (!(condition))                                                          \
        {                                                                          \
            LOG_FATAL(Logger::Nodes::ASSERTS, "Assertion failed: %s", #condition); \
            ABORT();                                                               \
        }                                                                          \
    }                                                                              \
    while (0)

#define ASSERT_COND_MSG(condition, msg)                                                        \
    do                                                                                         \
    {                                                                                          \
        if (!(condition))                                                                      \
        {                                                                                      \
            LOG_FATAL(Logger::Nodes::ASSERTS, "Assertion failed: (%s)\n %s", #condition, msg); \
            ABORT();                                                                           \
        }                                                                                      \
    }                                                                                          \
    while (0)

#define ASSERT_COND_MSG_ARGS(condition, format, ...)                                                       \
    do                                                                                                     \
    {                                                                                                      \
        if (!(condition))                                                                                  \
        {                                                                                                  \
            LOG_FATAL(Logger::Nodes::ASSERTS, "Assertion failed: %s\n" format, #condition, ##__VA_ARGS__); \
            ABORT();                                                                                       \
        }                                                                                                  \
    }                                                                                                      \
    while (0)

#elif defined(__linux__)  // defined(ARDUINO_ESP32S3_DEV)
namespace
{
    [[noreturn]] inline void ABORT(const std::string& message)
    {
        /* TODO Validate Ros Node assert showing*/
        std::cerr << "Process called abort() because: " << message << std::endl;
        abort();
    }
}  // namespace

// Implementation macros for Linux
#define ASSERT()                              \
    do                                        \
    {                                         \
        ABORT("Explicit assertion failure."); \
    }                                         \
    while (0)

#define ASSERT_MSG(msg)                                     \
    do                                                      \
    {                                                       \
        ABORT(std::string("Explicit assertion: ") + (msg)); \
    }                                                       \
    while (0)

#define ASSERT_MSG_ARGS(format, ...)                             \
    do                                                           \
    {                                                            \
        char buffer[256];                                        \
        snprintf(buffer, sizeof(buffer), format, ##__VA_ARGS__); \
        ABORT(std::string("Explicit assertion: ") + buffer);     \
    }                                                            \
    while (0)

#define ASSERT_COND(condition)                                     \
    do                                                             \
    {                                                              \
        if (!(condition))                                          \
        {                                                          \
            ABORT(std::string("Assertion failed: ") + #condition); \
        }                                                          \
    }                                                              \
    while (0)

#define ASSERT_COND_MSG(condition, msg)                                           \
    do                                                                            \
    {                                                                             \
        if (!(condition))                                                         \
        {                                                                         \
            ABORT(std::string("Assertion failed: ") + #condition + ", " + (msg)); \
        }                                                                         \
    }                                                                             \
    while (0)

#define ASSERT_COND_MSG_ARGS(condition, format, ...)                               \
    do                                                                             \
    {                                                                              \
        if (!(condition))                                                          \
        {                                                                          \
            char buffer[256];                                                      \
            snprintf(buffer, sizeof(buffer), format, ##__VA_ARGS__);               \
            ABORT(std::string("Assertion failed: ") + #condition + ", " + buffer); \
        }                                                                          \
    }                                                                              \
    while (0)

#endif  // defined(ARDUINO_ESP32S3_DEV)
#endif  // ASSERT_HPP
