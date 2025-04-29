#ifndef LOG_HPP
#define LOG_HPP

#if defined(VERBOSE)

#if !defined(GLOBAL_SEVERITY_LEVEL)
#define GLOBAL_SEVERITY_LEVEL Logger::eSeverityLevels::DEBUG_
#endif

#if !defined(NODE_BYPASS_SEVERITY_LEVEL)
#define NODE_BYPASS_SEVERITY_LEVEL Logger::eSeverityLevels::ERROR
#endif

#if defined(ARDUINO_ESP32S3_DEV)
#include <Stream.h>
#include <Arduino.h>
#elif defined(__linux__)
#include <cstdarg>
#endif  // defined(ARDUINO_ESP32S3_DEV)

#include "rover_lib2/helpers/time.hpp"
#include "rover_lib2/helpers/macros.hpp"
#include "rover_lib2/helpers/health_state.hpp"

/**
 * @brief Usage: DEFINE_LOG_NODE(Main, Logger::eNodeState::ON/OFF);
 *
 * @note Data can then be logged with LOG_INFO(Logger::Nodes::Main, "Hello world");
 */
#define DEFINE_LOG_NODE(name_, state_)                              \
    namespace Logger::Nodes                                         \
    {                                                               \
        inline constexpr Logger::Nodes::Node name_{#name_, state_}; \
    }
namespace Logger
{
#if defined(ARDUINO_ESP32S3_DEV)
    Stream& loggerStream __attribute__((weak)) = Serial;
#endif  // defined(ARDUINO_ESP32S3_DEV)

    enum class eSeverityLevels : uint8_t
    {
        DEBUG_ = 0,
        INFO,
        WARN,
        ERROR,
        FATAL
    };

    enum class eNodeState
    {
        ON,
        OFF
    };

    namespace Nodes
    {
        struct Node
        {
            constexpr Node(const char* name_, eNodeState state_):
                name(name_),
                state(state_)
            {
            }
            const char* const name;
            const eNodeState state;
        };
    }  // namespace Nodes

    namespace
    {
        constexpr const char* SEVERITY_NAMES[] = {"DEBUG", "INFO", "WARN", "ERROR", "FATAL"};
        constexpr const char* SEVERITY_COLORS[] = {"\033[90m", "\033[97m", "\033[33m", "\033[31m", "\033[31m"};
        constexpr const char* COLOR_RESET = "\033[97m";

        template<eSeverityLevels SEVERITY>
        inline void logInternal(const Nodes::Node& node, char* fileName, int lineNb, const char* format, ...)
        {
            if (TO_UNDERLYING(SEVERITY) >= static_cast<size_t>(GLOBAL_SEVERITY_LEVEL)
                && (TO_UNDERLYING(SEVERITY) >= static_cast<size_t>(NODE_BYPASS_SEVERITY_LEVEL)
                    || node.state == Logger::eNodeState::ON))
            {
                const auto severityIndex = TO_UNDERLYING(SEVERITY);
#if defined(ARDUINO_ESP32S3_DEV)
                loggerStream.printf("%s[%lu][%s]%s:%d: ",
                                    SEVERITY_COLORS[severityIndex],
                                    static_cast<uint32_t>(Time::millis()),
                                    SEVERITY_NAMES[severityIndex],
                                    fileName,
                                    lineNb);
                va_list args;
                va_start(args, format);
                loggerStream.vprintf(format, args);
                va_end(args);
                loggerStream.printf("\n%s", COLOR_RESET);

#elif defined(__linux__)
                printf("%s[%lu][%s]%s:%d: ",
                       SEVERITY_COLORS[severityIndex],
                       Time::millis(),
                       SEVERITY_NAMES[severityIndex],
                       fileName,
                       lineNb);
                va_list args;
                va_start(args, format);
                vprintf(format, args);
                va_end(args);
                printf("\n%s", COLOR_RESET);
#endif  // defined(ARDUINO_ESP32S3_DEV)
            }
        }
    }  // namespace
}  // namespace Logger

#define LOG_DEBUG(NODE, FMT, ...) \
    Logger::logInternal<Logger::eSeverityLevels::DEBUG_>(NODE, __FILENAME__, __LINE__, FMT, ##__VA_ARGS__)

#define LOG_INFO(NODE, FMT, ...) \
    Logger::logInternal<Logger::eSeverityLevels::INFO>(NODE, __FILENAME__, __LINE__, FMT, ##__VA_ARGS__)

#define LOG_WARN(NODE, FMT, ...) \
    Logger::logInternal<Logger::eSeverityLevels::WARN>(NODE, __FILENAME__, __LINE__, FMT, ##__VA_ARGS__)

#define LOG_ERROR(NODE, FMT, ...)                                                                          \
    Logger::logInternal<Logger::eSeverityLevels::ERROR>(NODE, __FILENAME__, __LINE__, FMT, ##__VA_ARGS__); \
    HealthState::getInstance().setInError();

#define LOG_FATAL(NODE, FMT, ...)                                                                          \
    Logger::logInternal<Logger::eSeverityLevels::FATAL>(NODE, __FILENAME__, __LINE__, FMT, ##__VA_ARGS__); \
    HealthState::getInstance().setInError();

/**
 * @brief Block execution until the Logger's TX buffer is empty
 *
 */
#define LOG_FLUSH() Logger::loggerStream.flush();

/**
 * @brief Default node for debug purposes
 *
 */
DEFINE_LOG_NODE(Debug, Logger::eNodeState::ON);

#else  // defined(VERBOSE)

#include "rover_lib2/helpers/health_state.hpp"

/* @brief usage example: DEFINE_LOG_NODE(<UNIQUE_NAME>, Logger::eNodeState::<VALUE>);
 */
#define DEFINE_LOG_NODE(name_, state_)

#define LOG_DEBUG(NODE, FMT, ...)
#define LOG_INFO(NODE, FMT, ...)
#define LOG_WARN(NODE, FMT, ...)
#define LOG_ERROR(NODE, FMT, ...) HealthState::getInstance().setInError()
#define LOG_FATAL(NODE, FMT, ...) HealthState::getInstance().setInError()

/**
 * @brief Block execution until the Logger's TX buffer is empty
 *
 */
#define LOG_FLUSH()

#endif  // defined(VERBOSE)

#endif  // LOG_HPP
