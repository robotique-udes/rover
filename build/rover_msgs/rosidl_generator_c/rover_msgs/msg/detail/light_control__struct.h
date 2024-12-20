// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rover_msgs:msg/LightControl.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__MSG__DETAIL__LIGHT_CONTROL__STRUCT_H_
#define ROVER_MSGS__MSG__DETAIL__LIGHT_CONTROL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'LIGHT'.
enum
{
  rover_msgs__msg__LightControl__LIGHT = 0
};

/// Constant 'LIGHT_INFRARED'.
enum
{
  rover_msgs__msg__LightControl__LIGHT_INFRARED = 1
};

/// Struct defined in msg/LightControl in the package rover_msgs.
typedef struct rover_msgs__msg__LightControl
{
  bool enable[2];
} rover_msgs__msg__LightControl;

// Struct for a sequence of rover_msgs__msg__LightControl.
typedef struct rover_msgs__msg__LightControl__Sequence
{
  rover_msgs__msg__LightControl * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_msgs__msg__LightControl__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROVER_MSGS__MSG__DETAIL__LIGHT_CONTROL__STRUCT_H_
