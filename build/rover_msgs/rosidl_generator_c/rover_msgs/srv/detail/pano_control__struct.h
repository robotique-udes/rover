// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rover_msgs:srv/PanoControl.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__SRV__DETAIL__PANO_CONTROL__STRUCT_H_
#define ROVER_MSGS__SRV__DETAIL__PANO_CONTROL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'ip_address'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/PanoControl in the package rover_msgs.
typedef struct rover_msgs__srv__PanoControl_Request
{
  bool start;
  bool stop;
  bool photo;
  rosidl_runtime_c__String ip_address;
} rover_msgs__srv__PanoControl_Request;

// Struct for a sequence of rover_msgs__srv__PanoControl_Request.
typedef struct rover_msgs__srv__PanoControl_Request__Sequence
{
  rover_msgs__srv__PanoControl_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_msgs__srv__PanoControl_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'status_message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/PanoControl in the package rover_msgs.
typedef struct rover_msgs__srv__PanoControl_Response
{
  bool success;
  rosidl_runtime_c__String status_message;
} rover_msgs__srv__PanoControl_Response;

// Struct for a sequence of rover_msgs__srv__PanoControl_Response.
typedef struct rover_msgs__srv__PanoControl_Response__Sequence
{
  rover_msgs__srv__PanoControl_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_msgs__srv__PanoControl_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROVER_MSGS__SRV__DETAIL__PANO_CONTROL__STRUCT_H_
