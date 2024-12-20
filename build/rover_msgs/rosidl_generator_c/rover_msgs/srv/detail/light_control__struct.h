// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rover_msgs:srv/LightControl.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__SRV__DETAIL__LIGHT_CONTROL__STRUCT_H_
#define ROVER_MSGS__SRV__DETAIL__LIGHT_CONTROL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'LIGHT'.
enum
{
  rover_msgs__srv__LightControl_Request__LIGHT = 0
};

/// Constant 'LIGHT_INFRARED'.
enum
{
  rover_msgs__srv__LightControl_Request__LIGHT_INFRARED = 1
};

/// Struct defined in srv/LightControl in the package rover_msgs.
typedef struct rover_msgs__srv__LightControl_Request
{
  uint8_t index;
  bool enable;
} rover_msgs__srv__LightControl_Request;

// Struct for a sequence of rover_msgs__srv__LightControl_Request.
typedef struct rover_msgs__srv__LightControl_Request__Sequence
{
  rover_msgs__srv__LightControl_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_msgs__srv__LightControl_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/LightControl in the package rover_msgs.
typedef struct rover_msgs__srv__LightControl_Response
{
  bool success;
} rover_msgs__srv__LightControl_Response;

// Struct for a sequence of rover_msgs__srv__LightControl_Response.
typedef struct rover_msgs__srv__LightControl_Response__Sequence
{
  rover_msgs__srv__LightControl_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_msgs__srv__LightControl_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROVER_MSGS__SRV__DETAIL__LIGHT_CONTROL__STRUCT_H_
