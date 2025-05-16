// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rover_msgs:msg/DrivetrainArbitration.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__MSG__DETAIL__DRIVETRAIN_ARBITRATION__STRUCT_H_
#define ROVER_MSGS__MSG__DETAIL__DRIVETRAIN_ARBITRATION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'NONE'.
enum
{
  rover_msgs__msg__DrivetrainArbitration__NONE = 0
};

/// Constant 'TELEOP'.
enum
{
  rover_msgs__msg__DrivetrainArbitration__TELEOP = 1
};

/// Constant 'AUTONOMUS'.
enum
{
  rover_msgs__msg__DrivetrainArbitration__AUTONOMUS = 2
};

/// Struct defined in msg/DrivetrainArbitration in the package rover_msgs.
typedef struct rover_msgs__msg__DrivetrainArbitration
{
  uint8_t arbitration;
} rover_msgs__msg__DrivetrainArbitration;

// Struct for a sequence of rover_msgs__msg__DrivetrainArbitration.
typedef struct rover_msgs__msg__DrivetrainArbitration__Sequence
{
  rover_msgs__msg__DrivetrainArbitration * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_msgs__msg__DrivetrainArbitration__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROVER_MSGS__MSG__DETAIL__DRIVETRAIN_ARBITRATION__STRUCT_H_
