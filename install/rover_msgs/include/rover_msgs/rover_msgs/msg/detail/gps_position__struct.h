// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rover_msgs:msg/GpsPosition.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__MSG__DETAIL__GPS_POSITION__STRUCT_H_
#define ROVER_MSGS__MSG__DETAIL__GPS_POSITION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/GpsPosition in the package rover_msgs.
typedef struct rover_msgs__msg__GpsPosition
{
  float latitude;
  float longitude;
} rover_msgs__msg__GpsPosition;

// Struct for a sequence of rover_msgs__msg__GpsPosition.
typedef struct rover_msgs__msg__GpsPosition__Sequence
{
  rover_msgs__msg__GpsPosition * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_msgs__msg__GpsPosition__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROVER_MSGS__MSG__DETAIL__GPS_POSITION__STRUCT_H_
