// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from rover_msgs:srv/NewGpsGoal.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "rover_msgs/srv/detail/new_gps_goal__rosidl_typesupport_introspection_c.h"
#include "rover_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rover_msgs/srv/detail/new_gps_goal__functions.h"
#include "rover_msgs/srv/detail/new_gps_goal__struct.h"


// Include directives for member types
// Member `waypoints`
#include "rover_msgs/msg/gps_position.h"
// Member `waypoints`
#include "rover_msgs/msg/detail/gps_position__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void rover_msgs__srv__NewGpsGoal_Request__rosidl_typesupport_introspection_c__NewGpsGoal_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rover_msgs__srv__NewGpsGoal_Request__init(message_memory);
}

void rover_msgs__srv__NewGpsGoal_Request__rosidl_typesupport_introspection_c__NewGpsGoal_Request_fini_function(void * message_memory)
{
  rover_msgs__srv__NewGpsGoal_Request__fini(message_memory);
}

size_t rover_msgs__srv__NewGpsGoal_Request__rosidl_typesupport_introspection_c__size_function__NewGpsGoal_Request__waypoints(
  const void * untyped_member)
{
  const rover_msgs__msg__GpsPosition__Sequence * member =
    (const rover_msgs__msg__GpsPosition__Sequence *)(untyped_member);
  return member->size;
}

const void * rover_msgs__srv__NewGpsGoal_Request__rosidl_typesupport_introspection_c__get_const_function__NewGpsGoal_Request__waypoints(
  const void * untyped_member, size_t index)
{
  const rover_msgs__msg__GpsPosition__Sequence * member =
    (const rover_msgs__msg__GpsPosition__Sequence *)(untyped_member);
  return &member->data[index];
}

void * rover_msgs__srv__NewGpsGoal_Request__rosidl_typesupport_introspection_c__get_function__NewGpsGoal_Request__waypoints(
  void * untyped_member, size_t index)
{
  rover_msgs__msg__GpsPosition__Sequence * member =
    (rover_msgs__msg__GpsPosition__Sequence *)(untyped_member);
  return &member->data[index];
}

void rover_msgs__srv__NewGpsGoal_Request__rosidl_typesupport_introspection_c__fetch_function__NewGpsGoal_Request__waypoints(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rover_msgs__msg__GpsPosition * item =
    ((const rover_msgs__msg__GpsPosition *)
    rover_msgs__srv__NewGpsGoal_Request__rosidl_typesupport_introspection_c__get_const_function__NewGpsGoal_Request__waypoints(untyped_member, index));
  rover_msgs__msg__GpsPosition * value =
    (rover_msgs__msg__GpsPosition *)(untyped_value);
  *value = *item;
}

void rover_msgs__srv__NewGpsGoal_Request__rosidl_typesupport_introspection_c__assign_function__NewGpsGoal_Request__waypoints(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rover_msgs__msg__GpsPosition * item =
    ((rover_msgs__msg__GpsPosition *)
    rover_msgs__srv__NewGpsGoal_Request__rosidl_typesupport_introspection_c__get_function__NewGpsGoal_Request__waypoints(untyped_member, index));
  const rover_msgs__msg__GpsPosition * value =
    (const rover_msgs__msg__GpsPosition *)(untyped_value);
  *item = *value;
}

bool rover_msgs__srv__NewGpsGoal_Request__rosidl_typesupport_introspection_c__resize_function__NewGpsGoal_Request__waypoints(
  void * untyped_member, size_t size)
{
  rover_msgs__msg__GpsPosition__Sequence * member =
    (rover_msgs__msg__GpsPosition__Sequence *)(untyped_member);
  rover_msgs__msg__GpsPosition__Sequence__fini(member);
  return rover_msgs__msg__GpsPosition__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember rover_msgs__srv__NewGpsGoal_Request__rosidl_typesupport_introspection_c__NewGpsGoal_Request_message_member_array[3] = {
  {
    "type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_msgs__srv__NewGpsGoal_Request, type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "index",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_msgs__srv__NewGpsGoal_Request, index),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "waypoints",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_msgs__srv__NewGpsGoal_Request, waypoints),  // bytes offset in struct
    NULL,  // default value
    rover_msgs__srv__NewGpsGoal_Request__rosidl_typesupport_introspection_c__size_function__NewGpsGoal_Request__waypoints,  // size() function pointer
    rover_msgs__srv__NewGpsGoal_Request__rosidl_typesupport_introspection_c__get_const_function__NewGpsGoal_Request__waypoints,  // get_const(index) function pointer
    rover_msgs__srv__NewGpsGoal_Request__rosidl_typesupport_introspection_c__get_function__NewGpsGoal_Request__waypoints,  // get(index) function pointer
    rover_msgs__srv__NewGpsGoal_Request__rosidl_typesupport_introspection_c__fetch_function__NewGpsGoal_Request__waypoints,  // fetch(index, &value) function pointer
    rover_msgs__srv__NewGpsGoal_Request__rosidl_typesupport_introspection_c__assign_function__NewGpsGoal_Request__waypoints,  // assign(index, value) function pointer
    rover_msgs__srv__NewGpsGoal_Request__rosidl_typesupport_introspection_c__resize_function__NewGpsGoal_Request__waypoints  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers rover_msgs__srv__NewGpsGoal_Request__rosidl_typesupport_introspection_c__NewGpsGoal_Request_message_members = {
  "rover_msgs__srv",  // message namespace
  "NewGpsGoal_Request",  // message name
  3,  // number of fields
  sizeof(rover_msgs__srv__NewGpsGoal_Request),
  rover_msgs__srv__NewGpsGoal_Request__rosidl_typesupport_introspection_c__NewGpsGoal_Request_message_member_array,  // message members
  rover_msgs__srv__NewGpsGoal_Request__rosidl_typesupport_introspection_c__NewGpsGoal_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  rover_msgs__srv__NewGpsGoal_Request__rosidl_typesupport_introspection_c__NewGpsGoal_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t rover_msgs__srv__NewGpsGoal_Request__rosidl_typesupport_introspection_c__NewGpsGoal_Request_message_type_support_handle = {
  0,
  &rover_msgs__srv__NewGpsGoal_Request__rosidl_typesupport_introspection_c__NewGpsGoal_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rover_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rover_msgs, srv, NewGpsGoal_Request)() {
  rover_msgs__srv__NewGpsGoal_Request__rosidl_typesupport_introspection_c__NewGpsGoal_Request_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rover_msgs, msg, GpsPosition)();
  if (!rover_msgs__srv__NewGpsGoal_Request__rosidl_typesupport_introspection_c__NewGpsGoal_Request_message_type_support_handle.typesupport_identifier) {
    rover_msgs__srv__NewGpsGoal_Request__rosidl_typesupport_introspection_c__NewGpsGoal_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &rover_msgs__srv__NewGpsGoal_Request__rosidl_typesupport_introspection_c__NewGpsGoal_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "rover_msgs/srv/detail/new_gps_goal__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rover_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "rover_msgs/srv/detail/new_gps_goal__functions.h"
// already included above
// #include "rover_msgs/srv/detail/new_gps_goal__struct.h"


// Include directives for member types
// Member `status`
#include "rosidl_runtime_c/string_functions.h"
// Member `route`
// already included above
// #include "rover_msgs/msg/gps_position.h"
// Member `route`
// already included above
// #include "rover_msgs/msg/detail/gps_position__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void rover_msgs__srv__NewGpsGoal_Response__rosidl_typesupport_introspection_c__NewGpsGoal_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rover_msgs__srv__NewGpsGoal_Response__init(message_memory);
}

void rover_msgs__srv__NewGpsGoal_Response__rosidl_typesupport_introspection_c__NewGpsGoal_Response_fini_function(void * message_memory)
{
  rover_msgs__srv__NewGpsGoal_Response__fini(message_memory);
}

size_t rover_msgs__srv__NewGpsGoal_Response__rosidl_typesupport_introspection_c__size_function__NewGpsGoal_Response__route(
  const void * untyped_member)
{
  const rover_msgs__msg__GpsPosition__Sequence * member =
    (const rover_msgs__msg__GpsPosition__Sequence *)(untyped_member);
  return member->size;
}

const void * rover_msgs__srv__NewGpsGoal_Response__rosidl_typesupport_introspection_c__get_const_function__NewGpsGoal_Response__route(
  const void * untyped_member, size_t index)
{
  const rover_msgs__msg__GpsPosition__Sequence * member =
    (const rover_msgs__msg__GpsPosition__Sequence *)(untyped_member);
  return &member->data[index];
}

void * rover_msgs__srv__NewGpsGoal_Response__rosidl_typesupport_introspection_c__get_function__NewGpsGoal_Response__route(
  void * untyped_member, size_t index)
{
  rover_msgs__msg__GpsPosition__Sequence * member =
    (rover_msgs__msg__GpsPosition__Sequence *)(untyped_member);
  return &member->data[index];
}

void rover_msgs__srv__NewGpsGoal_Response__rosidl_typesupport_introspection_c__fetch_function__NewGpsGoal_Response__route(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rover_msgs__msg__GpsPosition * item =
    ((const rover_msgs__msg__GpsPosition *)
    rover_msgs__srv__NewGpsGoal_Response__rosidl_typesupport_introspection_c__get_const_function__NewGpsGoal_Response__route(untyped_member, index));
  rover_msgs__msg__GpsPosition * value =
    (rover_msgs__msg__GpsPosition *)(untyped_value);
  *value = *item;
}

void rover_msgs__srv__NewGpsGoal_Response__rosidl_typesupport_introspection_c__assign_function__NewGpsGoal_Response__route(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rover_msgs__msg__GpsPosition * item =
    ((rover_msgs__msg__GpsPosition *)
    rover_msgs__srv__NewGpsGoal_Response__rosidl_typesupport_introspection_c__get_function__NewGpsGoal_Response__route(untyped_member, index));
  const rover_msgs__msg__GpsPosition * value =
    (const rover_msgs__msg__GpsPosition *)(untyped_value);
  *item = *value;
}

bool rover_msgs__srv__NewGpsGoal_Response__rosidl_typesupport_introspection_c__resize_function__NewGpsGoal_Response__route(
  void * untyped_member, size_t size)
{
  rover_msgs__msg__GpsPosition__Sequence * member =
    (rover_msgs__msg__GpsPosition__Sequence *)(untyped_member);
  rover_msgs__msg__GpsPosition__Sequence__fini(member);
  return rover_msgs__msg__GpsPosition__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember rover_msgs__srv__NewGpsGoal_Response__rosidl_typesupport_introspection_c__NewGpsGoal_Response_message_member_array[3] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_msgs__srv__NewGpsGoal_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_msgs__srv__NewGpsGoal_Response, status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "route",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_msgs__srv__NewGpsGoal_Response, route),  // bytes offset in struct
    NULL,  // default value
    rover_msgs__srv__NewGpsGoal_Response__rosidl_typesupport_introspection_c__size_function__NewGpsGoal_Response__route,  // size() function pointer
    rover_msgs__srv__NewGpsGoal_Response__rosidl_typesupport_introspection_c__get_const_function__NewGpsGoal_Response__route,  // get_const(index) function pointer
    rover_msgs__srv__NewGpsGoal_Response__rosidl_typesupport_introspection_c__get_function__NewGpsGoal_Response__route,  // get(index) function pointer
    rover_msgs__srv__NewGpsGoal_Response__rosidl_typesupport_introspection_c__fetch_function__NewGpsGoal_Response__route,  // fetch(index, &value) function pointer
    rover_msgs__srv__NewGpsGoal_Response__rosidl_typesupport_introspection_c__assign_function__NewGpsGoal_Response__route,  // assign(index, value) function pointer
    rover_msgs__srv__NewGpsGoal_Response__rosidl_typesupport_introspection_c__resize_function__NewGpsGoal_Response__route  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers rover_msgs__srv__NewGpsGoal_Response__rosidl_typesupport_introspection_c__NewGpsGoal_Response_message_members = {
  "rover_msgs__srv",  // message namespace
  "NewGpsGoal_Response",  // message name
  3,  // number of fields
  sizeof(rover_msgs__srv__NewGpsGoal_Response),
  rover_msgs__srv__NewGpsGoal_Response__rosidl_typesupport_introspection_c__NewGpsGoal_Response_message_member_array,  // message members
  rover_msgs__srv__NewGpsGoal_Response__rosidl_typesupport_introspection_c__NewGpsGoal_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  rover_msgs__srv__NewGpsGoal_Response__rosidl_typesupport_introspection_c__NewGpsGoal_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t rover_msgs__srv__NewGpsGoal_Response__rosidl_typesupport_introspection_c__NewGpsGoal_Response_message_type_support_handle = {
  0,
  &rover_msgs__srv__NewGpsGoal_Response__rosidl_typesupport_introspection_c__NewGpsGoal_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rover_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rover_msgs, srv, NewGpsGoal_Response)() {
  rover_msgs__srv__NewGpsGoal_Response__rosidl_typesupport_introspection_c__NewGpsGoal_Response_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rover_msgs, msg, GpsPosition)();
  if (!rover_msgs__srv__NewGpsGoal_Response__rosidl_typesupport_introspection_c__NewGpsGoal_Response_message_type_support_handle.typesupport_identifier) {
    rover_msgs__srv__NewGpsGoal_Response__rosidl_typesupport_introspection_c__NewGpsGoal_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &rover_msgs__srv__NewGpsGoal_Response__rosidl_typesupport_introspection_c__NewGpsGoal_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rover_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rover_msgs/srv/detail/new_gps_goal__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers rover_msgs__srv__detail__new_gps_goal__rosidl_typesupport_introspection_c__NewGpsGoal_service_members = {
  "rover_msgs__srv",  // service namespace
  "NewGpsGoal",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // rover_msgs__srv__detail__new_gps_goal__rosidl_typesupport_introspection_c__NewGpsGoal_Request_message_type_support_handle,
  NULL  // response message
  // rover_msgs__srv__detail__new_gps_goal__rosidl_typesupport_introspection_c__NewGpsGoal_Response_message_type_support_handle
};

static rosidl_service_type_support_t rover_msgs__srv__detail__new_gps_goal__rosidl_typesupport_introspection_c__NewGpsGoal_service_type_support_handle = {
  0,
  &rover_msgs__srv__detail__new_gps_goal__rosidl_typesupport_introspection_c__NewGpsGoal_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rover_msgs, srv, NewGpsGoal_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rover_msgs, srv, NewGpsGoal_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rover_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rover_msgs, srv, NewGpsGoal)() {
  if (!rover_msgs__srv__detail__new_gps_goal__rosidl_typesupport_introspection_c__NewGpsGoal_service_type_support_handle.typesupport_identifier) {
    rover_msgs__srv__detail__new_gps_goal__rosidl_typesupport_introspection_c__NewGpsGoal_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)rover_msgs__srv__detail__new_gps_goal__rosidl_typesupport_introspection_c__NewGpsGoal_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rover_msgs, srv, NewGpsGoal_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rover_msgs, srv, NewGpsGoal_Response)()->data;
  }

  return &rover_msgs__srv__detail__new_gps_goal__rosidl_typesupport_introspection_c__NewGpsGoal_service_type_support_handle;
}
