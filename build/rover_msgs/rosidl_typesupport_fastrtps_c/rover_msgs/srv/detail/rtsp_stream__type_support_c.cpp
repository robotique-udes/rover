// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from rover_msgs:srv/RtspStream.idl
// generated code does not contain a copyright notice
#include "rover_msgs/srv/detail/rtsp_stream__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rover_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "rover_msgs/srv/detail/rtsp_stream__struct.h"
#include "rover_msgs/srv/detail/rtsp_stream__functions.h"
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

#include "rosidl_runtime_c/string.h"  // stream_id
#include "rosidl_runtime_c/string_functions.h"  // stream_id

// forward declare type support functions


using _RtspStream_Request__ros_msg_type = rover_msgs__srv__RtspStream_Request;

static bool _RtspStream_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _RtspStream_Request__ros_msg_type * ros_message = static_cast<const _RtspStream_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: stream_id
  {
    const rosidl_runtime_c__String * str = &ros_message->stream_id;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: demand
  {
    cdr << (ros_message->demand ? true : false);
  }

  return true;
}

static bool _RtspStream_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _RtspStream_Request__ros_msg_type * ros_message = static_cast<_RtspStream_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: stream_id
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->stream_id.data) {
      rosidl_runtime_c__String__init(&ros_message->stream_id);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->stream_id,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'stream_id'\n");
      return false;
    }
  }

  // Field name: demand
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->demand = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rover_msgs
size_t get_serialized_size_rover_msgs__srv__RtspStream_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _RtspStream_Request__ros_msg_type * ros_message = static_cast<const _RtspStream_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name stream_id
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->stream_id.size + 1);
  // field.name demand
  {
    size_t item_size = sizeof(ros_message->demand);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _RtspStream_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_rover_msgs__srv__RtspStream_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rover_msgs
size_t max_serialized_size_rover_msgs__srv__RtspStream_Request(
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

  // member: stream_id
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: demand
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
    using DataType = rover_msgs__srv__RtspStream_Request;
    is_plain =
      (
      offsetof(DataType, demand) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _RtspStream_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_rover_msgs__srv__RtspStream_Request(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_RtspStream_Request = {
  "rover_msgs::srv",
  "RtspStream_Request",
  _RtspStream_Request__cdr_serialize,
  _RtspStream_Request__cdr_deserialize,
  _RtspStream_Request__get_serialized_size,
  _RtspStream_Request__max_serialized_size
};

static rosidl_message_type_support_t _RtspStream_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_RtspStream_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rover_msgs, srv, RtspStream_Request)() {
  return &_RtspStream_Request__type_support;
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
// #include "rover_msgs/srv/detail/rtsp_stream__struct.h"
// already included above
// #include "rover_msgs/srv/detail/rtsp_stream__functions.h"
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


using _RtspStream_Response__ros_msg_type = rover_msgs__srv__RtspStream_Response;

static bool _RtspStream_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _RtspStream_Response__ros_msg_type * ros_message = static_cast<const _RtspStream_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: success
  {
    cdr << (ros_message->success ? true : false);
  }

  return true;
}

static bool _RtspStream_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _RtspStream_Response__ros_msg_type * ros_message = static_cast<_RtspStream_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: success
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->success = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rover_msgs
size_t get_serialized_size_rover_msgs__srv__RtspStream_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _RtspStream_Response__ros_msg_type * ros_message = static_cast<const _RtspStream_Response__ros_msg_type *>(untyped_ros_message);
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

static uint32_t _RtspStream_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_rover_msgs__srv__RtspStream_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rover_msgs
size_t max_serialized_size_rover_msgs__srv__RtspStream_Response(
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
    using DataType = rover_msgs__srv__RtspStream_Response;
    is_plain =
      (
      offsetof(DataType, success) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _RtspStream_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_rover_msgs__srv__RtspStream_Response(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_RtspStream_Response = {
  "rover_msgs::srv",
  "RtspStream_Response",
  _RtspStream_Response__cdr_serialize,
  _RtspStream_Response__cdr_deserialize,
  _RtspStream_Response__get_serialized_size,
  _RtspStream_Response__max_serialized_size
};

static rosidl_message_type_support_t _RtspStream_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_RtspStream_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rover_msgs, srv, RtspStream_Response)() {
  return &_RtspStream_Response__type_support;
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
#include "rover_msgs/srv/rtsp_stream.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t RtspStream__callbacks = {
  "rover_msgs::srv",
  "RtspStream",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rover_msgs, srv, RtspStream_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rover_msgs, srv, RtspStream_Response)(),
};

static rosidl_service_type_support_t RtspStream__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &RtspStream__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rover_msgs, srv, RtspStream)() {
  return &RtspStream__handle;
}

#if defined(__cplusplus)
}
#endif
