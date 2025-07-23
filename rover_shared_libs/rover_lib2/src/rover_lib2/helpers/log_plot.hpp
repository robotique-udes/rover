#ifndef ROVER_LIB2_HELPERS_LOG_PLOT_HPP
#define ROVER_LIB2_HELPERS_LOG_PLOT_HPP

#include "rover_lib2/helpers/log.hpp"

#if defined(ARDUINO_ESP32S3_DEV) && defined(DEBUG)

#define _LOG_PLOT_PRINT_1(val1)                \
    Logger::loggerStream.print(">" #val1 ":"); \
    Logger::loggerStream.println(val1)

#define _LOG_PLOT_PRINT_2(val1, val2)          \
    _LOG_PLOT_PRINT_1(val1);                   \
    Logger::loggerStream.print(">" #val2 ":"); \
    Logger::loggerStream.println(val2)

#define _LOG_PLOT_PRINT_3(val1, val2, val3)    \
    _LOG_PLOT_PRINT_2(val1, val2);             \
    Logger::loggerStream.print(">" #val3 ":"); \
    Logger::loggerStream.println(val3)

#define _LOG_PLOT_PRINT_4(val1, val2, val3, val4) \
    _LOG_PLOT_PRINT_3(val1, val2, val3);          \
    Logger::loggerStream.print(">" #val4 ":");    \
    Logger::loggerStream.println(val4)

#define _LOG_PLOT_PRINT_5(val1, val2, val3, val4, val5) \
    _LOG_PLOT_PRINT_4(val1, val2, val3, val4);          \
    Logger::loggerStream.print(">" #val5 ":");          \
    Logger::loggerStream.println(val5)

#define _LOG_PLOT_PRINT_6(val1, val2, val3, val4, val5, val6) \
    _LOG_PLOT_PRINT_5(val1, val2, val3, val4, val5);          \
    Logger::loggerStream.print(">" #val6 ":");                \
    Logger::loggerStream.println(val6)

#define _GET_LOG_PLOT_MACRO(_1, _2, _3, _4, _5, _6, NAME, ...) NAME

#define LOG_PLOT(node, ...)                     \
    if ((node).state == Logger::eNodeState::ON) \
    {                                           \
        _GET_LOG_PLOT_MACRO(__VA_ARGS__,        \
                            _LOG_PLOT_PRINT_6,  \
                            _LOG_PLOT_PRINT_5,  \
                            _LOG_PLOT_PRINT_4,  \
                            _LOG_PLOT_PRINT_3,  \
                            _LOG_PLOT_PRINT_2,  \
                            _LOG_PLOT_PRINT_1)  \
        (__VA_ARGS__);                          \
    }

#else

#define LOG_PLOT(node, ...)

#endif

#endif
