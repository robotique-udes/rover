// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rover_msgs:srv/NewGpsGoal.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__SRV__DETAIL__NEW_GPS_GOAL__STRUCT_H_
#define ROVER_MSGS__SRV__DETAIL__NEW_GPS_GOAL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'GET_ROUTE'.
enum
{
  rover_msgs__srv__NewGpsGoal_Request__GET_ROUTE = 0
};

/// Constant 'NEW_ROUTE'.
enum
{
  rover_msgs__srv__NewGpsGoal_Request__NEW_ROUTE = 1
};

/// Constant 'NEW_GOAL_END_APPEND'.
enum
{
  rover_msgs__srv__NewGpsGoal_Request__NEW_GOAL_END_APPEND = 2
};

/// Constant 'NEW_GOAL_END_OVERWRITE'.
enum
{
  rover_msgs__srv__NewGpsGoal_Request__NEW_GOAL_END_OVERWRITE = 3
};

/// Constant 'NEW_WAYPOINT_BEFORE_END'.
enum
{
  rover_msgs__srv__NewGpsGoal_Request__NEW_WAYPOINT_BEFORE_END = 4
};

/// Constant 'NEW_WAYPOINT_INDEX_INSERT'.
enum
{
  rover_msgs__srv__NewGpsGoal_Request__NEW_WAYPOINT_INDEX_INSERT = 5
};

/// Constant 'NEW_WAYPOINT_INDEX_REPLACE'.
enum
{
  rover_msgs__srv__NewGpsGoal_Request__NEW_WAYPOINT_INDEX_REPLACE = 6
};

/// Constant 'CLEAR_WAYPOINT_INDEX'.
enum
{
  rover_msgs__srv__NewGpsGoal_Request__CLEAR_WAYPOINT_INDEX = 7
};

/// Constant 'CLEAR_ROUTE'.
enum
{
  rover_msgs__srv__NewGpsGoal_Request__CLEAR_ROUTE = 10
};

// Include directives for member types
// Member 'waypoints'
#include "rover_msgs/msg/detail/gps_position__struct.h"

/// Struct defined in srv/NewGpsGoal in the package rover_msgs.
typedef struct rover_msgs__srv__NewGpsGoal_Request
{
  uint8_t type;
  uint8_t index;
  rover_msgs__msg__GpsPosition__Sequence waypoints;
} rover_msgs__srv__NewGpsGoal_Request;

// Struct for a sequence of rover_msgs__srv__NewGpsGoal_Request.
typedef struct rover_msgs__srv__NewGpsGoal_Request__Sequence
{
  rover_msgs__srv__NewGpsGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_msgs__srv__NewGpsGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'status'
#include "rosidl_runtime_c/string.h"
// Member 'route'
// already included above
// #include "rover_msgs/msg/detail/gps_position__struct.h"

/// Struct defined in srv/NewGpsGoal in the package rover_msgs.
typedef struct rover_msgs__srv__NewGpsGoal_Response
{
  bool success;
  rosidl_runtime_c__String status;
  rover_msgs__msg__GpsPosition__Sequence route;
} rover_msgs__srv__NewGpsGoal_Response;

// Struct for a sequence of rover_msgs__srv__NewGpsGoal_Response.
typedef struct rover_msgs__srv__NewGpsGoal_Response__Sequence
{
  rover_msgs__srv__NewGpsGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_msgs__srv__NewGpsGoal_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROVER_MSGS__SRV__DETAIL__NEW_GPS_GOAL__STRUCT_H_
