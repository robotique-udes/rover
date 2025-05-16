// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from rover_msgs:srv/JoyDemuxSetState.idl
// generated code does not contain a copyright notice
#include "rover_msgs/srv/detail/joy_demux_set_state__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rover_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "rover_msgs/srv/detail/joy_demux_set_state__struct.h"
#include "rover_msgs/srv/detail/joy_demux_set_state__functions.h"
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


using _JoyDemuxSetState_Request__ros_msg_type = rover_msgs__srv__JoyDemuxSetState_Request;

static bool _JoyDemuxSetState_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _JoyDemuxSetState_Request__ros_msg_type * ros_message = static_cast<const _JoyDemuxSetState_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: controller_type
  {
    cdr << ros_message->controller_type;
  }

  // Field name: destination
  {
    cdr << ros_message->destination;
  }

  // Field name: force
  {
    cdr << (ros_message->force ? true : false);
  }

  return true;
}

static bool _JoyDemuxSetState_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _JoyDemuxSetState_Request__ros_msg_type * ros_message = static_cast<_JoyDemuxSetState_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: controller_type
  {
    cdr >> ros_message->controller_type;
  }

  // Field name: destination
  {
    cdr >> ros_message->destination;
  }

  // Field name: force
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->force = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rover_msgs
size_t get_serialized_size_rover_msgs__srv__JoyDemuxSetState_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _JoyDemuxSetState_Request__ros_msg_type * ros_message = static_cast<const _JoyDemuxSetState_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name controller_type
  {
    size_t item_size = sizeof(ros_message->controller_type);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name destination
  {
    size_t item_size = sizeof(ros_message->destination);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name force
  {
    size_t item_size = sizeof(ros_message->force);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _JoyDemuxSetState_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_rover_msgs__srv__JoyDemuxSetState_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rover_msgs
size_t max_serialized_size_rover_msgs__srv__JoyDemuxSetState_Request(
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

  // member: controller_type
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: destination
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: force
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
    using DataType = rover_msgs__srv__JoyDemuxSetState_Request;
    is_plain =
      (
      offsetof(DataType, force) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _JoyDemuxSetState_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_rover_msgs__srv__JoyDemuxSetState_Request(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_JoyDemuxSetState_Request = {
  "rover_msgs::srv",
  "JoyDemuxSetState_Request",
  _JoyDemuxSetState_Request__cdr_serialize,
  _JoyDemuxSetState_Request__cdr_deserialize,
  _JoyDemuxSetState_Request__get_serialized_size,
  _JoyDemuxSetState_Request__max_serialized_size
};

static rosidl_message_type_support_t _JoyDemuxSetState_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_JoyDemuxSetState_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rover_msgs, srv, JoyDemuxSetState_Request)() {
  return &_JoyDemuxSetState_Request__type_support;
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
// #include "rover_msgs/srv/detail/joy_demux_set_state__struct.h"
// already included above
// #include "rover_msgs/srv/detail/joy_demux_set_state__functions.h"
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


using _JoyDemuxSetState_Response__ros_msg_type = rover_msgs__srv__JoyDemuxSetState_Response;

static bool _JoyDemuxSetState_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _JoyDemuxSetState_Response__ros_msg_type * ros_message = static_cast<const _JoyDemuxSetState_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: success
  {
    cdr << (ros_message->success ? true : false);
  }

  return true;
}

static bool _JoyDemuxSetState_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _JoyDemuxSetState_Response__ros_msg_type * ros_message = static_cast<_JoyDemuxSetState_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: success
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->success = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rover_msgs
size_t get_serialized_size_rover_msgs__srv__JoyDemuxSetState_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _JoyDemuxSetState_Response__ros_msg_type * ros_message = static_cast<const _JoyDemuxSetState_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name success
  {
    size_t item_size = sizeof(ros_message->success);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _JoyDemuxSetState_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_rover_msgs__srv__JoyDemuxSetState_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rover_msgs
size_t max_serialized_size_rover_msgs__srv__JoyDemuxSetState_Response(
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

  // member: success
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
    using DataType = rover_msgs__srv__JoyDemuxSetState_Response;
    is_plain =
      (
      offsetof(DataType, success) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _JoyDemuxSetState_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_rover_msgs__srv__JoyDemuxSetState_Response(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_JoyDemuxSetState_Response = {
  "rover_msgs::srv",
  "JoyDemuxSetState_Response",
  _JoyDemuxSetState_Response__cdr_serialize,
  _JoyDemuxSetState_Response__cdr_deserialize,
  _JoyDemuxSetState_Response__get_serialized_size,
  _JoyDemuxSetState_Response__max_serialized_size
};

static rosidl_message_type_support_t _JoyDemuxSetState_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_JoyDemuxSetState_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rover_msgs, srv, JoyDemuxSetState_Response)() {
  return &_JoyDemuxSetState_Response__type_support;
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
#include "rover_msgs/srv/joy_demux_set_state.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t JoyDemuxSetState__callbacks = {
  "rover_msgs::srv",
  "JoyDemuxSetState",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rover_msgs, srv, JoyDemuxSetState_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rover_msgs, srv, JoyDemuxSetState_Response)(),
};

static rosidl_service_type_support_t JoyDemuxSetState__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &JoyDemuxSetState__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rover_msgs, srv, JoyDemuxSetState)() {
  return &JoyDemuxSetState__handle;
}

#if defined(__cplusplus)
}
#endif
