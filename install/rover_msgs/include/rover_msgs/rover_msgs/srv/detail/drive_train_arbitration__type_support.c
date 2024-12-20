// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from rover_msgs:srv/DriveTrainArbitration.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "rover_msgs/srv/detail/drive_train_arbitration__rosidl_typesupport_introspection_c.h"
#include "rover_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rover_msgs/srv/detail/drive_train_arbitration__functions.h"
#include "rover_msgs/srv/detail/drive_train_arbitration__struct.h"


// Include directives for member types
// Member `target_arbitration`
#include "rover_msgs/msg/drivetrain_arbitration.h"
// Member `target_arbitration`
#include "rover_msgs/msg/detail/drivetrain_arbitration__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void rover_msgs__srv__DriveTrainArbitration_Request__rosidl_typesupport_introspection_c__DriveTrainArbitration_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rover_msgs__srv__DriveTrainArbitration_Request__init(message_memory);
}

void rover_msgs__srv__DriveTrainArbitration_Request__rosidl_typesupport_introspection_c__DriveTrainArbitration_Request_fini_function(void * message_memory)
{
  rover_msgs__srv__DriveTrainArbitration_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember rover_msgs__srv__DriveTrainArbitration_Request__rosidl_typesupport_introspection_c__DriveTrainArbitration_Request_message_member_array[1] = {
  {
    "target_arbitration",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_msgs__srv__DriveTrainArbitration_Request, target_arbitration),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers rover_msgs__srv__DriveTrainArbitration_Request__rosidl_typesupport_introspection_c__DriveTrainArbitration_Request_message_members = {
  "rover_msgs__srv",  // message namespace
  "DriveTrainArbitration_Request",  // message name
  1,  // number of fields
  sizeof(rover_msgs__srv__DriveTrainArbitration_Request),
  rover_msgs__srv__DriveTrainArbitration_Request__rosidl_typesupport_introspection_c__DriveTrainArbitration_Request_message_member_array,  // message members
  rover_msgs__srv__DriveTrainArbitration_Request__rosidl_typesupport_introspection_c__DriveTrainArbitration_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  rover_msgs__srv__DriveTrainArbitration_Request__rosidl_typesupport_introspection_c__DriveTrainArbitration_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t rover_msgs__srv__DriveTrainArbitration_Request__rosidl_typesupport_introspection_c__DriveTrainArbitration_Request_message_type_support_handle = {
  0,
  &rover_msgs__srv__DriveTrainArbitration_Request__rosidl_typesupport_introspection_c__DriveTrainArbitration_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rover_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rover_msgs, srv, DriveTrainArbitration_Request)() {
  rover_msgs__srv__DriveTrainArbitration_Request__rosidl_typesupport_introspection_c__DriveTrainArbitration_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rover_msgs, msg, DrivetrainArbitration)();
  if (!rover_msgs__srv__DriveTrainArbitration_Request__rosidl_typesupport_introspection_c__DriveTrainArbitration_Request_message_type_support_handle.typesupport_identifier) {
    rover_msgs__srv__DriveTrainArbitration_Request__rosidl_typesupport_introspection_c__DriveTrainArbitration_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &rover_msgs__srv__DriveTrainArbitration_Request__rosidl_typesupport_introspection_c__DriveTrainArbitration_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "rover_msgs/srv/detail/drive_train_arbitration__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rover_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "rover_msgs/srv/detail/drive_train_arbitration__functions.h"
// already included above
// #include "rover_msgs/srv/detail/drive_train_arbitration__struct.h"


// Include directives for member types
// Member `current_arbitration`
// already included above
// #include "rover_msgs/msg/drivetrain_arbitration.h"
// Member `current_arbitration`
// already included above
// #include "rover_msgs/msg/detail/drivetrain_arbitration__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void rover_msgs__srv__DriveTrainArbitration_Response__rosidl_typesupport_introspection_c__DriveTrainArbitration_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rover_msgs__srv__DriveTrainArbitration_Response__init(message_memory);
}

void rover_msgs__srv__DriveTrainArbitration_Response__rosidl_typesupport_introspection_c__DriveTrainArbitration_Response_fini_function(void * message_memory)
{
  rover_msgs__srv__DriveTrainArbitration_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember rover_msgs__srv__DriveTrainArbitration_Response__rosidl_typesupport_introspection_c__DriveTrainArbitration_Response_message_member_array[1] = {
  {
    "current_arbitration",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_msgs__srv__DriveTrainArbitration_Response, current_arbitration),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers rover_msgs__srv__DriveTrainArbitration_Response__rosidl_typesupport_introspection_c__DriveTrainArbitration_Response_message_members = {
  "rover_msgs__srv",  // message namespace
  "DriveTrainArbitration_Response",  // message name
  1,  // number of fields
  sizeof(rover_msgs__srv__DriveTrainArbitration_Response),
  rover_msgs__srv__DriveTrainArbitration_Response__rosidl_typesupport_introspection_c__DriveTrainArbitration_Response_message_member_array,  // message members
  rover_msgs__srv__DriveTrainArbitration_Response__rosidl_typesupport_introspection_c__DriveTrainArbitration_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  rover_msgs__srv__DriveTrainArbitration_Response__rosidl_typesupport_introspection_c__DriveTrainArbitration_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t rover_msgs__srv__DriveTrainArbitration_Response__rosidl_typesupport_introspection_c__DriveTrainArbitration_Response_message_type_support_handle = {
  0,
  &rover_msgs__srv__DriveTrainArbitration_Response__rosidl_typesupport_introspection_c__DriveTrainArbitration_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rover_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rover_msgs, srv, DriveTrainArbitration_Response)() {
  rover_msgs__srv__DriveTrainArbitration_Response__rosidl_typesupport_introspection_c__DriveTrainArbitration_Response_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rover_msgs, msg, DrivetrainArbitration)();
  if (!rover_msgs__srv__DriveTrainArbitration_Response__rosidl_typesupport_introspection_c__DriveTrainArbitration_Response_message_type_support_handle.typesupport_identifier) {
    rover_msgs__srv__DriveTrainArbitration_Response__rosidl_typesupport_introspection_c__DriveTrainArbitration_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &rover_msgs__srv__DriveTrainArbitration_Response__rosidl_typesupport_introspection_c__DriveTrainArbitration_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rover_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rover_msgs/srv/detail/drive_train_arbitration__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers rover_msgs__srv__detail__drive_train_arbitration__rosidl_typesupport_introspection_c__DriveTrainArbitration_service_members = {
  "rover_msgs__srv",  // service namespace
  "DriveTrainArbitration",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // rover_msgs__srv__detail__drive_train_arbitration__rosidl_typesupport_introspection_c__DriveTrainArbitration_Request_message_type_support_handle,
  NULL  // response message
  // rover_msgs__srv__detail__drive_train_arbitration__rosidl_typesupport_introspection_c__DriveTrainArbitration_Response_message_type_support_handle
};

static rosidl_service_type_support_t rover_msgs__srv__detail__drive_train_arbitration__rosidl_typesupport_introspection_c__DriveTrainArbitration_service_type_support_handle = {
  0,
  &rover_msgs__srv__detail__drive_train_arbitration__rosidl_typesupport_introspection_c__DriveTrainArbitration_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rover_msgs, srv, DriveTrainArbitration_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rover_msgs, srv, DriveTrainArbitration_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rover_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rover_msgs, srv, DriveTrainArbitration)() {
  if (!rover_msgs__srv__detail__drive_train_arbitration__rosidl_typesupport_introspection_c__DriveTrainArbitration_service_type_support_handle.typesupport_identifier) {
    rover_msgs__srv__detail__drive_train_arbitration__rosidl_typesupport_introspection_c__DriveTrainArbitration_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)rover_msgs__srv__detail__drive_train_arbitration__rosidl_typesupport_introspection_c__DriveTrainArbitration_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rover_msgs, srv, DriveTrainArbitration_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rover_msgs, srv, DriveTrainArbitration_Response)()->data;
  }

  return &rover_msgs__srv__detail__drive_train_arbitration__rosidl_typesupport_introspection_c__DriveTrainArbitration_service_type_support_handle;
}
