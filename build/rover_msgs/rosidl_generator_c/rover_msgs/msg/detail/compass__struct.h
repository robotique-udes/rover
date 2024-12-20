// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rover_msgs:msg/Compass.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__MSG__DETAIL__COMPASS__STRUCT_H_
#define ROVER_MSGS__MSG__DETAIL__COMPASS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Compass in the package rover_msgs.
typedef struct rover_msgs__msg__Compass
{
  float heading;
  float pitch;
} rover_msgs__msg__Compass;

// Struct for a sequence of rover_msgs__msg__Compass.
typedef struct rover_msgs__msg__Compass__Sequence
{
  rover_msgs__msg__Compass * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_msgs__msg__Compass__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROVER_MSGS__MSG__DETAIL__COMPASS__STRUCT_H_
