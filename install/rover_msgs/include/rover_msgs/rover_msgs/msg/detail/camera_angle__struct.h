// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rover_msgs:msg/CameraAngle.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__MSG__DETAIL__CAMERA_ANGLE__STRUCT_H_
#define ROVER_MSGS__MSG__DETAIL__CAMERA_ANGLE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/CameraAngle in the package rover_msgs.
typedef struct rover_msgs__msg__CameraAngle
{
  float angle;
} rover_msgs__msg__CameraAngle;

// Struct for a sequence of rover_msgs__msg__CameraAngle.
typedef struct rover_msgs__msg__CameraAngle__Sequence
{
  rover_msgs__msg__CameraAngle * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_msgs__msg__CameraAngle__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROVER_MSGS__MSG__DETAIL__CAMERA_ANGLE__STRUCT_H_
