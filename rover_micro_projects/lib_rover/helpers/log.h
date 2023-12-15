#ifndef __LOG_H__
#define __LOG_H__

#include "helpers/logger.h"

#ifdef VERBOSE
// Args: (Logger::eLoggerLevel), (string literal)NodeName, (string literal)format, vars
#define LOG(severity, ...)                                                                      \
    {                                                                                           \
        if (Logger.isAlive())                                                                   \
        {                                                                                       \
            Logger.log(severity, __FILENAME__, __FUNCTION__, __LINE__, __VA_ARGS__);            \
        }                                                                                       \
        else                                                                                    \
        {                                                                                       \
            if (severity > LOGGER_LOWEST_LEVEL)                                                 \
            {                                                                                   \
                char severityStr[6] = "0_0";                                                    \
                char colorStr[8] = "\033[97m";                                                  \
                switch (severity)                                                               \
                {                                                                               \
                case DEBUG:                                                                     \
                    strcpy(severityStr, "DEBUG");                                               \
                    break;                                                                      \
                                                                                                \
                case INFO:                                                                      \
                    strcpy(severityStr, "INFO");                                                \
                    break;                                                                      \
                                                                                                \
                case WARN:                                                                      \
                    strcpy(severityStr, "WARN");                                                \
                    strcpy(colorStr, "\033[33m");                                               \
                    break;                                                                      \
                                                                                                \
                case ERROR:                                                                     \
                    strcpy(severityStr, "ERROR");                                               \
                    strcpy(colorStr, "\033[31m");                                               \
                    break;                                                                      \
                                                                                                \
                case FATAL:                                                                     \
                    strcpy(severityStr, "FATAL");                                               \
                    strcpy(colorStr, "\033[31m");                                               \
                    break;                                                                      \
                }                                                                               \
                                                                                                \
                Serial.printf("%s[%s]%s(%d): ", colorStr, severityStr, __FILENAME__, __LINE__); \
                Serial.printf(__VA_ARGS__);                                                     \
                Serial.printf("\n\033[97m");                                                    \
            }                                                                                   \
        }                                                                                       \
    }
#else
#define LOG(severity, ...)
#endif // VERBOSE

#endif //__LOG_H__
