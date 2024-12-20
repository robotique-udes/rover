// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rover_msgs:msg/CameraControl.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__MSG__DETAIL__CAMERA_CONTROL__STRUCT_H_
#define ROVER_MSGS__MSG__DETAIL__CAMERA_CONTROL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'CAM_A2'.
enum
{
  rover_msgs__msg__CameraControl__CAM_A2 = 0
};

/// Constant 'CAM_R1M_1'.
enum
{
  rover_msgs__msg__CameraControl__CAM_R1M_1 = 1
};

/// Constant 'CAM_R1M_2'.
enum
{
  rover_msgs__msg__CameraControl__CAM_R1M_2 = 2
};

/// Constant 'CAM_R1M_3'.
enum
{
  rover_msgs__msg__CameraControl__CAM_R1M_3 = 3
};

/// Struct defined in msg/CameraControl in the package rover_msgs.
typedef struct rover_msgs__msg__CameraControl
{
  bool enable[4];
} rover_msgs__msg__CameraControl;

// Struct for a sequence of rover_msgs__msg__CameraControl.
typedef struct rover_msgs__msg__CameraControl__Sequence
{
  rover_msgs__msg__CameraControl * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_msgs__msg__CameraControl__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROVER_MSGS__MSG__DETAIL__CAMERA_CONTROL__STRUCT_H_
