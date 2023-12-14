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

#define GET_ARRAY_SIZE(a) (sizeof(a) / sizeof(a[0]))
#define FOR_ALL(x) for (uint8_t i = 0; i < GET_ARRAY_SIZE(x); i++)

#define GET_WORST_OF(a, b) ((a == false || b == false) ? false : true)

// DO NOT USE
// Do a while loop or, even better, a for(EVER) loop in your setup() function
// This will keep all setup variables inside the scope and will remove the need
// to declare them globally
void loop() {}

#endif //(__MACRO_H__)
