// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from rover_msgs:srv/AntennaArbitration.idl
// generated code does not contain a copyright notice
#include "rover_msgs/srv/detail/antenna_arbitration__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rover_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "rover_msgs/srv/detail/antenna_arbitration__struct.h"
#include "rover_msgs/srv/detail/antenna_arbitration__functions.h"
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


// forward declare type support functions


using _AntennaArbitration_Request__ros_msg_type = rover_msgs__srv__AntennaArbitration_Request;

static bool _AntennaArbitration_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _AntennaArbitration_Request__ros_msg_type * ros_message = static_cast<const _AntennaArbitration_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: target_arbitration
  {
    cdr << ros_message->target_arbitration;
  }

  return true;
}

static bool _AntennaArbitration_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _AntennaArbitration_Request__ros_msg_type * ros_message = static_cast<_AntennaArbitration_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: target_arbitration
  {
    cdr >> ros_message->target_arbitration;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rover_msgs
size_t get_serialized_size_rover_msgs__srv__AntennaArbitration_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _AntennaArbitration_Request__ros_msg_type * ros_message = static_cast<const _AntennaArbitration_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name target_arbitration
  {
    size_t item_size = sizeof(ros_message->target_arbitration);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _AntennaArbitration_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_rover_msgs__srv__AntennaArbitration_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rover_msgs
size_t max_serialized_size_rover_msgs__srv__AntennaArbitration_Request(
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

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = rover_msgs__srv__AntennaArbitration_Request;
    is_plain =
      (
      offsetof(DataType, target_arbitration) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _AntennaArbitration_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_rover_msgs__srv__AntennaArbitration_Request(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_AntennaArbitration_Request = {
  "rover_msgs::srv",
  "AntennaArbitration_Request",
  _AntennaArbitration_Request__cdr_serialize,
  _AntennaArbitration_Request__cdr_deserialize,
  _AntennaArbitration_Request__get_serialized_size,
  _AntennaArbitration_Request__max_serialized_size
};

static rosidl_message_type_support_t _AntennaArbitration_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_AntennaArbitration_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rover_msgs, srv, AntennaArbitration_Request)() {
  return &_AntennaArbitration_Request__type_support;
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
// #include "rover_msgs/srv/detail/antenna_arbitration__struct.h"
// already included above
// #include "rover_msgs/srv/detail/antenna_arbitration__functions.h"
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


// forward declare type support functions


using _AntennaArbitration_Response__ros_msg_type = rover_msgs__srv__AntennaArbitration_Response;

static bool _AntennaArbitration_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _AntennaArbitration_Response__ros_msg_type * ros_message = static_cast<const _AntennaArbitration_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: current_arbitration
  {
    cdr << ros_message->current_arbitration;
  }

  return true;
}

static bool _AntennaArbitration_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _AntennaArbitration_Response__ros_msg_type * ros_message = static_cast<_AntennaArbitration_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: current_arbitration
  {
    cdr >> ros_message->current_arbitration;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rover_msgs
size_t get_serialized_size_rover_msgs__srv__AntennaArbitration_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _AntennaArbitration_Response__ros_msg_type * ros_message = static_cast<const _AntennaArbitration_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name current_arbitration
  {
    size_t item_size = sizeof(ros_message->current_arbitration);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _AntennaArbitration_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_rover_msgs__srv__AntennaArbitration_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rover_msgs
size_t max_serialized_size_rover_msgs__srv__AntennaArbitration_Response(
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

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = rover_msgs__srv__AntennaArbitration_Response;
    is_plain =
      (
      offsetof(DataType, current_arbitration) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _AntennaArbitration_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_rover_msgs__srv__AntennaArbitration_Response(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_AntennaArbitration_Response = {
  "rover_msgs::srv",
  "AntennaArbitration_Response",
  _AntennaArbitration_Response__cdr_serialize,
  _AntennaArbitration_Response__cdr_deserialize,
  _AntennaArbitration_Response__get_serialized_size,
  _AntennaArbitration_Response__max_serialized_size
};

static rosidl_message_type_support_t _AntennaArbitration_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_AntennaArbitration_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rover_msgs, srv, AntennaArbitration_Response)() {
  return &_AntennaArbitration_Response__type_support;
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
#include "rover_msgs/srv/antenna_arbitration.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t AntennaArbitration__callbacks = {
  "rover_msgs::srv",
  "AntennaArbitration",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rover_msgs, srv, AntennaArbitration_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rover_msgs, srv, AntennaArbitration_Response)(),
};

static rosidl_service_type_support_t AntennaArbitration__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &AntennaArbitration__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rover_msgs, srv, AntennaArbitration)() {
  return &AntennaArbitration__handle;
}

#if defined(__cplusplus)
}
#endif
