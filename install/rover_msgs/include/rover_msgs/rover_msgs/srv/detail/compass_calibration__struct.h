// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rover_msgs:srv/CompassCalibration.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__SRV__DETAIL__COMPASS_CALIBRATION__STRUCT_H_
#define ROVER_MSGS__SRV__DETAIL__COMPASS_CALIBRATION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/CompassCalibration in the package rover_msgs.
typedef struct rover_msgs__srv__CompassCalibration_Request
{
  int16_t angle_offset;
} rover_msgs__srv__CompassCalibration_Request;

// Struct for a sequence of rover_msgs__srv__CompassCalibration_Request.
typedef struct rover_msgs__srv__CompassCalibration_Request__Sequence
{
  rover_msgs__srv__CompassCalibration_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_msgs__srv__CompassCalibration_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/CompassCalibration in the package rover_msgs.
typedef struct rover_msgs__srv__CompassCalibration_Response
{
  bool success;
} rover_msgs__srv__CompassCalibration_Response;

// Struct for a sequence of rover_msgs__srv__CompassCalibration_Response.
typedef struct rover_msgs__srv__CompassCalibration_Response__Sequence
{
  rover_msgs__srv__CompassCalibration_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_msgs__srv__CompassCalibration_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROVER_MSGS__SRV__DETAIL__COMPASS_CALIBRATION__STRUCT_H_
