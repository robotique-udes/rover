// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rover_msgs:msg/Gps.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__MSG__DETAIL__GPS__STRUCT_H_
#define ROVER_MSGS__MSG__DETAIL__GPS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Gps in the package rover_msgs.
/**
  * From GPS
 */
typedef struct rover_msgs__msg__Gps
{
  float latitude;
  float longitude;
  float height;
  float heading_gps;
  float heading_track;
  float speed;
  uint8_t satellite;
  /// From magnetometer
  float heading;
} rover_msgs__msg__Gps;

// Struct for a sequence of rover_msgs__msg__Gps.
typedef struct rover_msgs__msg__Gps__Sequence
{
  rover_msgs__msg__Gps * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_msgs__msg__Gps__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROVER_MSGS__MSG__DETAIL__GPS__STRUCT_H_
