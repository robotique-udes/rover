// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from rover_msgs:srv/DriveTrainArbitration.idl
// generated code does not contain a copyright notice
#include "rover_msgs/srv/detail/drive_train_arbitration__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rover_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "rover_msgs/srv/detail/drive_train_arbitration__struct.h"
#include "rover_msgs/srv/detail/drive_train_arbitration__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rover_msgs/msg/detail/drivetrain_arbitration__functions.h"  // target_arbitration

// forward declare type support functions
size_t get_serialized_size_rover_msgs__msg__DrivetrainArbitration(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_rover_msgs__msg__DrivetrainArbitration(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rover_msgs, msg, DrivetrainArbitration)();


using _DriveTrainArbitration_Request__ros_msg_type = rover_msgs__srv__DriveTrainArbitration_Request;

static bool _DriveTrainArbitration_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _DriveTrainArbitration_Request__ros_msg_type * ros_message = static_cast<const _DriveTrainArbitration_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: target_arbitration
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, rover_msgs, msg, DrivetrainArbitration
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->target_arbitration, cdr))
    {
      return false;
    }
  }

  return true;
}

static bool _DriveTrainArbitration_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _DriveTrainArbitration_Request__ros_msg_type * ros_message = static_cast<_DriveTrainArbitration_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: target_arbitration
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, rover_msgs, msg, DrivetrainArbitration
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->target_arbitration))
    {
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rover_msgs
size_t get_serialized_size_rover_msgs__srv__DriveTrainArbitration_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _DriveTrainArbitration_Request__ros_msg_type * ros_message = static_cast<const _DriveTrainArbitration_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name target_arbitration

  current_alignment += get_serialized_size_rover_msgs__msg__DrivetrainArbitration(
    &(ros_message->target_arbitration), current_alignment);

  return current_alignment - initial_alignment;
}

static uint32_t _DriveTrainArbitration_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_rover_msgs__srv__DriveTrainArbitration_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rover_msgs
size_t max_serialized_size_rover_msgs__srv__DriveTrainArbitration_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: target_arbitration
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_rover_msgs__msg__DrivetrainArbitration(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = rover_msgs__srv__DriveTrainArbitration_Request;
    is_plain =
      (
      offsetof(DataType, target_arbitration) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _DriveTrainArbitration_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_rover_msgs__srv__DriveTrainArbitration_Request(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_DriveTrainArbitration_Request = {
  "rover_msgs::srv",
  "DriveTrainArbitration_Request",
  _DriveTrainArbitration_Request__cdr_serialize,
  _DriveTrainArbitration_Request__cdr_deserialize,
  _DriveTrainArbitration_Request__get_serialized_size,
  _DriveTrainArbitration_Request__max_serialized_size
};

static rosidl_message_type_support_t _DriveTrainArbitration_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_DriveTrainArbitration_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rover_msgs, srv, DriveTrainArbitration_Request)() {
  return &_DriveTrainArbitration_Request__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "rover_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "rover_msgs/srv/detail/drive_train_arbitration__struct.h"
// already included above
// #include "rover_msgs/srv/detail/drive_train_arbitration__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

// already included above
// #include "rover_msgs/msg/detail/drivetrain_arbitration__functions.h"  // current_arbitration

// forward declare type support functions
size_t get_serialized_size_rover_msgs__msg__DrivetrainArbitration(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_rover_msgs__msg__DrivetrainArbitration(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rover_msgs, msg, DrivetrainArbitration)();


using _DriveTrainArbitration_Response__ros_msg_type = rover_msgs__srv__DriveTrainArbitration_Response;

static bool _DriveTrainArbitration_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _DriveTrainArbitration_Response__ros_msg_type * ros_message = static_cast<const _DriveTrainArbitration_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: current_arbitration
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, rover_msgs, msg, DrivetrainArbitration
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->current_arbitration, cdr))
    {
      return false;
    }
  }

  return true;
}

static bool _DriveTrainArbitration_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _DriveTrainArbitration_Response__ros_msg_type * ros_message = static_cast<_DriveTrainArbitration_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: current_arbitration
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, rover_msgs, msg, DrivetrainArbitration
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->current_arbitration))
    {
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rover_msgs
size_t get_serialized_size_rover_msgs__srv__DriveTrainArbitration_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _DriveTrainArbitration_Response__ros_msg_type * ros_message = static_cast<const _DriveTrainArbitration_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name current_arbitration

  current_alignment += get_serialized_size_rover_msgs__msg__DrivetrainArbitration(
    &(ros_message->current_arbitration), current_alignment);

  return current_alignment - initial_alignment;
}

static uint32_t _DriveTrainArbitration_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_rover_msgs__srv__DriveTrainArbitration_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rover_msgs
size_t max_serialized_size_rover_msgs__srv__DriveTrainArbitration_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: current_arbitration
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_rover_msgs__msg__DrivetrainArbitration(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = rover_msgs__srv__DriveTrainArbitration_Response;
    is_plain =
      (
      offsetof(DataType, current_arbitration) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _DriveTrainArbitration_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_rover_msgs__srv__DriveTrainArbitration_Response(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_DriveTrainArbitration_Response = {
  "rover_msgs::srv",
  "DriveTrainArbitration_Response",
  _DriveTrainArbitration_Response__cdr_serialize,
  _DriveTrainArbitration_Response__cdr_deserialize,
  _DriveTrainArbitration_Response__get_serialized_size,
  _DriveTrainArbitration_Response__max_serialized_size
};

static rosidl_message_type_support_t _DriveTrainArbitration_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_DriveTrainArbitration_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rover_msgs, srv, DriveTrainArbitration_Response)() {
  return &_DriveTrainArbitration_Response__type_support;
}

#if defined(__cplusplus)
}
#endif

#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rover_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "rover_msgs/srv/drive_train_arbitration.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t DriveTrainArbitration__callbacks = {
  "rover_msgs::srv",
  "DriveTrainArbitration",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rover_msgs, srv, DriveTrainArbitration_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rover_msgs, srv, DriveTrainArbitration_Response)(),
};

static rosidl_service_type_support_t DriveTrainArbitration__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &DriveTrainArbitration__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rover_msgs, srv, DriveTrainArbitration)() {
  return &DriveTrainArbitration__handle;
}

#if defined(__cplusplus)
}
#endif
