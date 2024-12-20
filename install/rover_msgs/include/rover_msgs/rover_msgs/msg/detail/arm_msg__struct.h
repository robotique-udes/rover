// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rover_msgs:msg/ArmMsg.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__MSG__DETAIL__ARM_MSG__STRUCT_H_
#define ROVER_MSGS__MSG__DETAIL__ARM_MSG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'JL'.
enum
{
  rover_msgs__msg__ArmMsg__JL = 0
};

/// Constant 'J0'.
enum
{
  rover_msgs__msg__ArmMsg__J0 = 1
};

/// Constant 'J1'.
enum
{
  rover_msgs__msg__ArmMsg__J1 = 2
};

/// Constant 'J2'.
enum
{
  rover_msgs__msg__ArmMsg__J2 = 3
};

/// Constant 'GRIPPER_TILT'.
enum
{
  rover_msgs__msg__ArmMsg__GRIPPER_TILT = 4
};

/// Constant 'GRIPPER_ROT'.
enum
{
  rover_msgs__msg__ArmMsg__GRIPPER_ROT = 5
};

/// Constant 'GRIPPER_CLOSE'.
enum
{
  rover_msgs__msg__ArmMsg__GRIPPER_CLOSE = 6
};

/// Struct defined in msg/ArmMsg in the package rover_msgs.
typedef struct rover_msgs__msg__ArmMsg
{
  float data[7];
} rover_msgs__msg__ArmMsg;

// Struct for a sequence of rover_msgs__msg__ArmMsg.
typedef struct rover_msgs__msg__ArmMsg__Sequence
{
  rover_msgs__msg__ArmMsg * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_msgs__msg__ArmMsg__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROVER_MSGS__MSG__DETAIL__ARM_MSG__STRUCT_H_
