// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rover_msgs:msg/PropulsionMotor.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__MSG__DETAIL__PROPULSION_MOTOR__STRUCT_H_
#define ROVER_MSGS__MSG__DETAIL__PROPULSION_MOTOR__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'FRONT_LEFT'.
enum
{
  rover_msgs__msg__PropulsionMotor__FRONT_LEFT = 0
};

/// Constant 'FRONT_RIGHT'.
enum
{
  rover_msgs__msg__PropulsionMotor__FRONT_RIGHT = 1
};

/// Constant 'REAR_LEFT'.
enum
{
  rover_msgs__msg__PropulsionMotor__REAR_LEFT = 2
};

/// Constant 'REAR_RIGHT'.
enum
{
  rover_msgs__msg__PropulsionMotor__REAR_RIGHT = 3
};

/// Struct defined in msg/PropulsionMotor in the package rover_msgs.
typedef struct rover_msgs__msg__PropulsionMotor
{
  bool enable[4];
  float target_speed[4];
  float current_speed[4];
  bool close_loop[4];
} rover_msgs__msg__PropulsionMotor;

// Struct for a sequence of rover_msgs__msg__PropulsionMotor.
typedef struct rover_msgs__msg__PropulsionMotor__Sequence
{
  rover_msgs__msg__PropulsionMotor * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_msgs__msg__PropulsionMotor__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROVER_MSGS__MSG__DETAIL__PROPULSION_MOTOR__STRUCT_H_
