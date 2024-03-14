#ifndef __MACROS_H__
#define __MACROS_H__

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

#define MAP(result_type, value, from_min, from_max, to_min, to_max) \
    (result_type)(((float)value - (float)from_min) / ((float)from_max - (float)from_min)) * ((float)to_max - (float)to_min) + (float)to_min

// Returns sign as a float value
#define SIGN(VAR) ((float)VAR > 0.0f ? 1.0f: -1.0f)

#define PI 3.14159265359

#define IN
#define OUT
#define INOUT

// Removes unused argument warning 
#define REMOVE_UNUSED(x) (void)(x)
    
#endif //__MACROS_H__
