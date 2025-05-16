// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from rover_msgs:srv/PanoControl.idl
// generated code does not contain a copyright notice
#include "rover_msgs/srv/detail/pano_control__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rover_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "rover_msgs/srv/detail/pano_control__struct.h"
#include "rover_msgs/srv/detail/pano_control__functions.h"
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

#include "rosidl_runtime_c/string.h"  // ip_address
#include "rosidl_runtime_c/string_functions.h"  // ip_address

// forward declare type support functions


using _PanoControl_Request__ros_msg_type = rover_msgs__srv__PanoControl_Request;

static bool _PanoControl_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _PanoControl_Request__ros_msg_type * ros_message = static_cast<const _PanoControl_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: start
  {
    cdr << (ros_message->start ? true : false);
  }

  // Field name: stop
  {
    cdr << (ros_message->stop ? true : false);
  }

  // Field name: photo
  {
    cdr << (ros_message->photo ? true : false);
  }

  // Field name: ip_address
  {
    const rosidl_runtime_c__String * str = &ros_message->ip_address;
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

  return true;
}

static bool _PanoControl_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _PanoControl_Request__ros_msg_type * ros_message = static_cast<_PanoControl_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: start
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->start = tmp ? true : false;
  }

  // Field name: stop
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->stop = tmp ? true : false;
  }

  // Field name: photo
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->photo = tmp ? true : false;
  }

  // Field name: ip_address
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->ip_address.data) {
      rosidl_runtime_c__String__init(&ros_message->ip_address);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->ip_address,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'ip_address'\n");
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rover_msgs
size_t get_serialized_size_rover_msgs__srv__PanoControl_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _PanoControl_Request__ros_msg_type * ros_message = static_cast<const _PanoControl_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name start
  {
    size_t item_size = sizeof(ros_message->start);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name stop
  {
    size_t item_size = sizeof(ros_message->stop);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name photo
  {
    size_t item_size = sizeof(ros_message->photo);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name ip_address
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->ip_address.size + 1);

  return current_alignment - initial_alignment;
}

static uint32_t _PanoControl_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_rover_msgs__srv__PanoControl_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rover_msgs
size_t max_serialized_size_rover_msgs__srv__PanoControl_Request(
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

  // member: start
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: stop
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: photo
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: ip_address
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

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = rover_msgs__srv__PanoControl_Request;
    is_plain =
      (
      offsetof(DataType, ip_address) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _PanoControl_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_rover_msgs__srv__PanoControl_Request(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_PanoControl_Request = {
  "rover_msgs::srv",
  "PanoControl_Request",
  _PanoControl_Request__cdr_serialize,
  _PanoControl_Request__cdr_deserialize,
  _PanoControl_Request__get_serialized_size,
  _PanoControl_Request__max_serialized_size
};

static rosidl_message_type_support_t _PanoControl_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_PanoControl_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rover_msgs, srv, PanoControl_Request)() {
  return &_PanoControl_Request__type_support;
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
// #include "rover_msgs/srv/detail/pano_control__struct.h"
// already included above
// #include "rover_msgs/srv/detail/pano_control__functions.h"
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
// #include "rosidl_runtime_c/string.h"  // status_message
// already included above
// #include "rosidl_runtime_c/string_functions.h"  // status_message

// forward declare type support functions


using _PanoControl_Response__ros_msg_type = rover_msgs__srv__PanoControl_Response;

static bool _PanoControl_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _PanoControl_Response__ros_msg_type * ros_message = static_cast<const _PanoControl_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: success
  {
    cdr << (ros_message->success ? true : false);
  }

  // Field name: status_message
  {
    const rosidl_runtime_c__String * str = &ros_message->status_message;
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

  return true;
}

static bool _PanoControl_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _PanoControl_Response__ros_msg_type * ros_message = static_cast<_PanoControl_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: success
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->success = tmp ? true : false;
  }

  // Field name: status_message
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->status_message.data) {
      rosidl_runtime_c__String__init(&ros_message->status_message);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->status_message,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'status_message'\n");
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rover_msgs
size_t get_serialized_size_rover_msgs__srv__PanoControl_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _PanoControl_Response__ros_msg_type * ros_message = static_cast<const _PanoControl_Response__ros_msg_type *>(untyped_ros_message);
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
  // field.name status_message
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->status_message.size + 1);

  return current_alignment - initial_alignment;
}

static uint32_t _PanoControl_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_rover_msgs__srv__PanoControl_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rover_msgs
size_t max_serialized_size_rover_msgs__srv__PanoControl_Response(
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
  // member: status_message
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

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = rover_msgs__srv__PanoControl_Response;
    is_plain =
      (
      offsetof(DataType, status_message) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _PanoControl_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_rover_msgs__srv__PanoControl_Response(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_PanoControl_Response = {
  "rover_msgs::srv",
  "PanoControl_Response",
  _PanoControl_Response__cdr_serialize,
  _PanoControl_Response__cdr_deserialize,
  _PanoControl_Response__get_serialized_size,
  _PanoControl_Response__max_serialized_size
};

static rosidl_message_type_support_t _PanoControl_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_PanoControl_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rover_msgs, srv, PanoControl_Response)() {
  return &_PanoControl_Response__type_support;
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
#include "rover_msgs/srv/pano_control.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t PanoControl__callbacks = {
  "rover_msgs::srv",
  "PanoControl",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rover_msgs, srv, PanoControl_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rover_msgs, srv, PanoControl_Response)(),
};

static rosidl_service_type_support_t PanoControl__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &PanoControl__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rover_msgs, srv, PanoControl)() {
  return &PanoControl__handle;
}

#if defined(__cplusplus)
}
#endif
