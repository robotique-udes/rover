// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rover_msgs:msg/CanDeviceStatus.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__MSG__DETAIL__CAN_DEVICE_STATUS__STRUCT_H_
#define ROVER_MSGS__MSG__DETAIL__CAN_DEVICE_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'STATUS_OK'.
enum
{
  rover_msgs__msg__CanDeviceStatus__STATUS_OK = 0
};

/// Constant 'STATUS_WARNING'.
enum
{
  rover_msgs__msg__CanDeviceStatus__STATUS_WARNING = 1
};

/// Constant 'STATUS_ERROR'.
enum
{
  rover_msgs__msg__CanDeviceStatus__STATUS_ERROR = 2
};

/// Struct defined in msg/CanDeviceStatus in the package rover_msgs.
typedef struct rover_msgs__msg__CanDeviceStatus
{
  uint16_t id;
  uint8_t error_state;
  bool watchdog_ok;
} rover_msgs__msg__CanDeviceStatus;

// Struct for a sequence of rover_msgs__msg__CanDeviceStatus.
typedef struct rover_msgs__msg__CanDeviceStatus__Sequence
{
  rover_msgs__msg__CanDeviceStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_msgs__msg__CanDeviceStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROVER_MSGS__MSG__DETAIL__CAN_DEVICE_STATUS__STRUCT_H_
