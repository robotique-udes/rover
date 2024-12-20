// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rover_msgs:msg/JoyDemuxStatus.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__MSG__DETAIL__JOY_DEMUX_STATUS__STRUCT_H_
#define ROVER_MSGS__MSG__DETAIL__JOY_DEMUX_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'CONTROLLER_MAIN'.
enum
{
  rover_msgs__msg__JoyDemuxStatus__CONTROLLER_MAIN = 0
};

/// Constant 'CONTROLLER_SECONDARY'.
enum
{
  rover_msgs__msg__JoyDemuxStatus__CONTROLLER_SECONDARY = 1
};

/// Constant 'DEST_DRIVE_TRAIN'.
/**
  * Possible demux destinations
 */
enum
{
  rover_msgs__msg__JoyDemuxStatus__DEST_DRIVE_TRAIN = 0
};

/// Constant 'DEST_ARM'.
enum
{
  rover_msgs__msg__JoyDemuxStatus__DEST_ARM = 1
};

/// Constant 'DEST_ANTENNA'.
enum
{
  rover_msgs__msg__JoyDemuxStatus__DEST_ANTENNA = 2
};

/// Constant 'DEST_NONE'.
enum
{
  rover_msgs__msg__JoyDemuxStatus__DEST_NONE = 3
};

/// Struct defined in msg/JoyDemuxStatus in the package rover_msgs.
typedef struct rover_msgs__msg__JoyDemuxStatus
{
  uint8_t controller_main_topic;
  uint8_t controller_secondary_topic;
} rover_msgs__msg__JoyDemuxStatus;

// Struct for a sequence of rover_msgs__msg__JoyDemuxStatus.
typedef struct rover_msgs__msg__JoyDemuxStatus__Sequence
{
  rover_msgs__msg__JoyDemuxStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_msgs__msg__JoyDemuxStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROVER_MSGS__MSG__DETAIL__JOY_DEMUX_STATUS__STRUCT_H_
