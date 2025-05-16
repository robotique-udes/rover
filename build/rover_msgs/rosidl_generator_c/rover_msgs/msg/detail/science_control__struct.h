// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rover_msgs:msg/ScienceControl.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__MSG__DETAIL__SCIENCE_CONTROL__STRUCT_H_
#define ROVER_MSGS__MSG__DETAIL__SCIENCE_CONTROL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'DOWN'.
enum
{
  rover_msgs__msg__ScienceControl__DOWN = 0
};

/// Constant 'UP'.
enum
{
  rover_msgs__msg__ScienceControl__UP = 1
};

/// Constant 'E1'.
enum
{
  rover_msgs__msg__ScienceControl__E1 = 0
};

/// Constant 'E2'.
enum
{
  rover_msgs__msg__ScienceControl__E2 = 1
};

/// Constant 'E3'.
enum
{
  rover_msgs__msg__ScienceControl__E3 = 2
};

/// Struct defined in msg/ScienceControl in the package rover_msgs.
typedef struct rover_msgs__msg__ScienceControl
{
  int8_t cmd;
  int8_t current_sample;
  bool dig;
} rover_msgs__msg__ScienceControl;

// Struct for a sequence of rover_msgs__msg__ScienceControl.
typedef struct rover_msgs__msg__ScienceControl__Sequence
{
  rover_msgs__msg__ScienceControl * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_msgs__msg__ScienceControl__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROVER_MSGS__MSG__DETAIL__SCIENCE_CONTROL__STRUCT_H_
