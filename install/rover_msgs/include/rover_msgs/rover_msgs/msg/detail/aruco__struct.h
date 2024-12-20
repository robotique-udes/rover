// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rover_msgs:msg/Aruco.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__MSG__DETAIL__ARUCO__STRUCT_H_
#define ROVER_MSGS__MSG__DETAIL__ARUCO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'id'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/Aruco in the package rover_msgs.
typedef struct rover_msgs__msg__Aruco
{
  bool valid;
  rosidl_runtime_c__uint8__Sequence id;
} rover_msgs__msg__Aruco;

// Struct for a sequence of rover_msgs__msg__Aruco.
typedef struct rover_msgs__msg__Aruco__Sequence
{
  rover_msgs__msg__Aruco * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_msgs__msg__Aruco__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROVER_MSGS__MSG__DETAIL__ARUCO__STRUCT_H_
