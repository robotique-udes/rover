#ifndef __MACRO_H__
#define __MACRO_H__

#include "Arduino.h"

// Like a while(true) but better
#define EVER \
    ;        \
    ;

// Use this macro to removed unused warnings when compiling
#define REMOVE_WARN_UNUSED(fn) \
    {                          \
        auto garbage = fn;     \
    }

#define RCLC_RET_ON_ERR(fn)          \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
            return false;            \
        }                            \
    }

// #define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

#define WARN(...)                                                 \
    {                                                             \
        Serial.printf("[*WARN]%s(%d): ", __FILENAME__, __LINE__); \
        Serial.printf(__VA_ARGS__);                               \
        Serial.printf("\n");                                      \
    }

#endif //(__MACRO_H__)
