// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rover_msgs:msg/Joy.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__MSG__DETAIL__JOY__STRUCT_H_
#define ROVER_MSGS__MSG__DETAIL__JOY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'JOYSTICK_LEFT_FRONT'.
enum
{
  rover_msgs__msg__Joy__JOYSTICK_LEFT_FRONT = 0
};

/// Constant 'JOYSTICK_LEFT_SIDE'.
enum
{
  rover_msgs__msg__Joy__JOYSTICK_LEFT_SIDE = 1
};

/// Constant 'JOYSTICK_LEFT_PUSH'.
enum
{
  rover_msgs__msg__Joy__JOYSTICK_LEFT_PUSH = 2
};

/// Constant 'JOYSTICK_RIGHT_FRONT'.
enum
{
  rover_msgs__msg__Joy__JOYSTICK_RIGHT_FRONT = 3
};

/// Constant 'JOYSTICK_RIGHT_SIDE'.
enum
{
  rover_msgs__msg__Joy__JOYSTICK_RIGHT_SIDE = 4
};

/// Constant 'JOYSTICK_RIGHT_PUSH'.
enum
{
  rover_msgs__msg__Joy__JOYSTICK_RIGHT_PUSH = 5
};

/// Constant 'CROSS_UP'.
enum
{
  rover_msgs__msg__Joy__CROSS_UP = 6
};

/// Constant 'CROSS_DOWN'.
enum
{
  rover_msgs__msg__Joy__CROSS_DOWN = 7
};

/// Constant 'CROSS_LEFT'.
enum
{
  rover_msgs__msg__Joy__CROSS_LEFT = 8
};

/// Constant 'CROSS_RIGHT'.
enum
{
  rover_msgs__msg__Joy__CROSS_RIGHT = 9
};

/// Constant 'L1'.
enum
{
  rover_msgs__msg__Joy__L1 = 10
};

/// Constant 'L2'.
enum
{
  rover_msgs__msg__Joy__L2 = 11
};

/// Constant 'R1'.
enum
{
  rover_msgs__msg__Joy__R1 = 12
};

/// Constant 'R2'.
enum
{
  rover_msgs__msg__Joy__R2 = 13
};

/// Constant 'A'.
enum
{
  rover_msgs__msg__Joy__A = 14
};

/// Constant 'B'.
enum
{
  rover_msgs__msg__Joy__B = 15
};

/// Constant 'X'.
enum
{
  rover_msgs__msg__Joy__X = 16
};

/// Constant 'Y'.
enum
{
  rover_msgs__msg__Joy__Y = 17
};

/// Constant 'EXT0'.
enum
{
  rover_msgs__msg__Joy__EXT0 = 18
};

/// Constant 'EXT1'.
enum
{
  rover_msgs__msg__Joy__EXT1 = 19
};

/// Constant 'EXT2'.
enum
{
  rover_msgs__msg__Joy__EXT2 = 20
};

/// Struct defined in msg/Joy in the package rover_msgs.
typedef struct rover_msgs__msg__Joy
{
  float joy_data[20];
} rover_msgs__msg__Joy;

// Struct for a sequence of rover_msgs__msg__Joy.
typedef struct rover_msgs__msg__Joy__Sequence
{
  rover_msgs__msg__Joy * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_msgs__msg__Joy__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROVER_MSGS__MSG__DETAIL__JOY__STRUCT_H_
