// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rover_msgs:srv/JoyDemuxSetState.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__SRV__DETAIL__JOY_DEMUX_SET_STATE__STRUCT_H_
#define ROVER_MSGS__SRV__DETAIL__JOY_DEMUX_SET_STATE__STRUCT_H_

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
  rover_msgs__srv__JoyDemuxSetState_Request__CONTROLLER_MAIN = 0
};

/// Constant 'CONTROLLER_SECONDARY'.
enum
{
  rover_msgs__srv__JoyDemuxSetState_Request__CONTROLLER_SECONDARY = 1
};

/// Constant 'DEST_DRIVE_TRAIN'.
/**
  * Possible demux destinations
 */
enum
{
  rover_msgs__srv__JoyDemuxSetState_Request__DEST_DRIVE_TRAIN = 0
};

/// Constant 'DEST_ARM'.
enum
{
  rover_msgs__srv__JoyDemuxSetState_Request__DEST_ARM = 1
};

/// Constant 'DEST_ANTENNA'.
enum
{
  rover_msgs__srv__JoyDemuxSetState_Request__DEST_ANTENNA = 2
};

/// Constant 'DEST_NONE'.
enum
{
  rover_msgs__srv__JoyDemuxSetState_Request__DEST_NONE = 3
};

/// Struct defined in srv/JoyDemuxSetState in the package rover_msgs.
typedef struct rover_msgs__srv__JoyDemuxSetState_Request
{
  uint8_t controller_type;
  uint8_t destination;
  bool force;
} rover_msgs__srv__JoyDemuxSetState_Request;

// Struct for a sequence of rover_msgs__srv__JoyDemuxSetState_Request.
typedef struct rover_msgs__srv__JoyDemuxSetState_Request__Sequence
{
  rover_msgs__srv__JoyDemuxSetState_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_msgs__srv__JoyDemuxSetState_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/JoyDemuxSetState in the package rover_msgs.
typedef struct rover_msgs__srv__JoyDemuxSetState_Response
{
  bool success;
} rover_msgs__srv__JoyDemuxSetState_Response;

// Struct for a sequence of rover_msgs__srv__JoyDemuxSetState_Response.
typedef struct rover_msgs__srv__JoyDemuxSetState_Response__Sequence
{
  rover_msgs__srv__JoyDemuxSetState_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_msgs__srv__JoyDemuxSetState_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROVER_MSGS__SRV__DETAIL__JOY_DEMUX_SET_STATE__STRUCT_H_
