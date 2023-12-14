#ifndef __LOG_H__
#define __LOG_H__

#include "helpers/logger.h"

#ifdef VERBOSE
// Args: (Logger::eLoggerLevel), (string literal)NodeName, (string literal)format, vars
#define LOG(severity, ...)                                                          \
    {                                                                               \
        if (Logger.isAlive())                                                       \
        {                                                                           \
            Logger.log(severity, __FILENAME__, "", __LINE__, __VA_ARGS__);          \
        }                                                                           \
        else                                                                        \
        {                                                                           \
            if (severity > LOGGER_LOWEST_LEVEL)                                     \
            {                                                                       \
                char severityStr[6] = "0_0";                                        \
                switch (severity)                                                   \
                {                                                                   \
                case DEBUG:                                                         \
                    strcpy(severityStr, "DEBUG");                                   \
                    break;                                                          \
                                                                                    \
                case INFO:                                                          \
                    strcpy(severityStr, "INFO");                                    \
                    break;                                                          \
                                                                                    \
                case WARN:                                                          \
                    strcpy(severityStr, "WARN");                                    \
                    break;                                                          \
                                                                                    \
                case ERROR:                                                         \
                    strcpy(severityStr, "ERROR");                                   \
                    break;                                                          \
                                                                                    \
                case FATAL:                                                         \
                    strcpy(severityStr, "FATAL");                                   \
                    break;                                                          \
                }                                                                   \
                                                                                    \
                Serial.printf("[%s]%s(%d): ", severityStr, __FILENAME__, __LINE__); \
                Serial.printf(__VA_ARGS__);                                         \
                Serial.printf("\n");                                                \
            }                                                                       \
        }                                                                           \
    }
#else
#define LOG(severity, ...)
#endif // VERBOSE

#endif //__LOG_H__
