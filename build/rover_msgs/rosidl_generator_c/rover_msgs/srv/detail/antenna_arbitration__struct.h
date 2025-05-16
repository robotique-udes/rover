// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rover_msgs:srv/AntennaArbitration.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__SRV__DETAIL__ANTENNA_ARBITRATION__STRUCT_H_
#define ROVER_MSGS__SRV__DETAIL__ANTENNA_ARBITRATION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'NOT_MOVING'.
enum
{
  rover_msgs__srv__AntennaArbitration_Request__NOT_MOVING = 0
};

/// Constant 'TELEOP'.
enum
{
  rover_msgs__srv__AntennaArbitration_Request__TELEOP = 1
};

/// Constant 'AUTONOMUS'.
enum
{
  rover_msgs__srv__AntennaArbitration_Request__AUTONOMUS = 2
};

/// Struct defined in srv/AntennaArbitration in the package rover_msgs.
typedef struct rover_msgs__srv__AntennaArbitration_Request
{
  uint8_t target_arbitration;
} rover_msgs__srv__AntennaArbitration_Request;

// Struct for a sequence of rover_msgs__srv__AntennaArbitration_Request.
typedef struct rover_msgs__srv__AntennaArbitration_Request__Sequence
{
  rover_msgs__srv__AntennaArbitration_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_msgs__srv__AntennaArbitration_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/AntennaArbitration in the package rover_msgs.
typedef struct rover_msgs__srv__AntennaArbitration_Response
{
  uint8_t current_arbitration;
} rover_msgs__srv__AntennaArbitration_Response;

// Struct for a sequence of rover_msgs__srv__AntennaArbitration_Response.
typedef struct rover_msgs__srv__AntennaArbitration_Response__Sequence
{
  rover_msgs__srv__AntennaArbitration_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_msgs__srv__AntennaArbitration_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROVER_MSGS__SRV__DETAIL__ANTENNA_ARBITRATION__STRUCT_H_
