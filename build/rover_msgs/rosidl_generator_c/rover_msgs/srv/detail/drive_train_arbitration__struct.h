// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rover_msgs:srv/DriveTrainArbitration.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__SRV__DETAIL__DRIVE_TRAIN_ARBITRATION__STRUCT_H_
#define ROVER_MSGS__SRV__DETAIL__DRIVE_TRAIN_ARBITRATION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'target_arbitration'
#include "rover_msgs/msg/detail/drivetrain_arbitration__struct.h"

/// Struct defined in srv/DriveTrainArbitration in the package rover_msgs.
typedef struct rover_msgs__srv__DriveTrainArbitration_Request
{
  rover_msgs__msg__DrivetrainArbitration target_arbitration;
} rover_msgs__srv__DriveTrainArbitration_Request;

// Struct for a sequence of rover_msgs__srv__DriveTrainArbitration_Request.
typedef struct rover_msgs__srv__DriveTrainArbitration_Request__Sequence
{
  rover_msgs__srv__DriveTrainArbitration_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_msgs__srv__DriveTrainArbitration_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'current_arbitration'
// already included above
// #include "rover_msgs/msg/detail/drivetrain_arbitration__struct.h"

/// Struct defined in srv/DriveTrainArbitration in the package rover_msgs.
typedef struct rover_msgs__srv__DriveTrainArbitration_Response
{
  rover_msgs__msg__DrivetrainArbitration current_arbitration;
} rover_msgs__srv__DriveTrainArbitration_Response;

// Struct for a sequence of rover_msgs__srv__DriveTrainArbitration_Response.
typedef struct rover_msgs__srv__DriveTrainArbitration_Response__Sequence
{
  rover_msgs__srv__DriveTrainArbitration_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_msgs__srv__DriveTrainArbitration_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROVER_MSGS__SRV__DETAIL__DRIVE_TRAIN_ARBITRATION__STRUCT_H_
