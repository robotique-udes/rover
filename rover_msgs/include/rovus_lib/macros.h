#ifndef __MACROS_H__
#define __MACROS_H__

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>

#define LOGGER this->get_logger()
#define CLOCK *this->get_clock()

// Return array size from first point
#define GET_ARRAY_SIZE(a) (sizeof(a) / sizeof(a[0]))

// The for(EVER) loop, a thousand times better than a while(true) loop
#define EVER \
    ;        \
    ;

//  Loop thru all element of passed array, use i as counter
//
//  Exemple:
//  FOR_ALL(array)
//  {
//      array[i] = smtg;
//  }
#define FOR_ALL(x) for (uint8_t i = 0; i < GET_ARRAY_SIZE(x); i++)

//  Apply dead zone around a value
//
//  Exemple for PID goal:
//  if(!IN_ERROR(PID.currentPos, 0.5f, PID.goalPos))
//  {
//      PID.calculateCmd();
//  }
#define IN_ERROR(VAR, ERROR, GOAL) \
    ((abs(VAR) < (abs(GOAL) + ERROR) && abs(VAR) > (abs(GOAL) - ERROR)))

#define MAP(RESULT_TYPE, VALUE, FROM_MIN, FROM_MAX, TO_MIN, TO_MAX) \
    (RESULT_TYPE)(((float)VALUE - (float)FROM_MIN) / ((float)FROM_MAX - (float)FROM_MIN)) * ((float)TO_MAX - (float)TO_MIN) + (float)TO_MIN

//Returns a value between set constrains
#define CONSTRAIN(VALUE, LOWER_RANGE, UPPER_RANGE) \
    VALUE > UPPER_RANGE ? UPPER_RANGE: (VALUE < LOWER_RANGE ? LOWER_RANGE : VALUE)

// Returns sign as a float value
#define SIGN(VAR) ((float)VAR > 0.0f ? 1.0f: -1.0f)

#define PI 3.14159265359

#define IN
#define OUT
#define INOUT

// Removes unused argument warning 
#define REMOVE_UNUSED(x) (void)(x)

#define GET_PACKAGE_SOURCE_DIR(package_name) \
    (ament_index_cpp::get_package_prefix(package_name) + "/../../src/rover/" + package_name)
    
#endif //__MACROS_H__
