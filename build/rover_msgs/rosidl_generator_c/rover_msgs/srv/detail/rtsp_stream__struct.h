// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rover_msgs:srv/RtspStream.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__SRV__DETAIL__RTSP_STREAM__STRUCT_H_
#define ROVER_MSGS__SRV__DETAIL__RTSP_STREAM__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'stream_id'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/RtspStream in the package rover_msgs.
typedef struct rover_msgs__srv__RtspStream_Request
{
  rosidl_runtime_c__String stream_id;
  bool demand;
} rover_msgs__srv__RtspStream_Request;

// Struct for a sequence of rover_msgs__srv__RtspStream_Request.
typedef struct rover_msgs__srv__RtspStream_Request__Sequence
{
  rover_msgs__srv__RtspStream_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_msgs__srv__RtspStream_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/RtspStream in the package rover_msgs.
typedef struct rover_msgs__srv__RtspStream_Response
{
  bool success;
} rover_msgs__srv__RtspStream_Response;

// Struct for a sequence of rover_msgs__srv__RtspStream_Response.
typedef struct rover_msgs__srv__RtspStream_Response__Sequence
{
  rover_msgs__srv__RtspStream_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_msgs__srv__RtspStream_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROVER_MSGS__SRV__DETAIL__RTSP_STREAM__STRUCT_H_
