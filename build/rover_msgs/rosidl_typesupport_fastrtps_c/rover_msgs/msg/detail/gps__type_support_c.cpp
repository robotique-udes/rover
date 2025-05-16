// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from rover_msgs:msg/Gps.idl
// generated code does not contain a copyright notice
#include "rover_msgs/msg/detail/gps__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rover_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "rover_msgs/msg/detail/gps__struct.h"
#include "rover_msgs/msg/detail/gps__functions.h"
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


using _Gps__ros_msg_type = rover_msgs__msg__Gps;

static bool _Gps__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _Gps__ros_msg_type * ros_message = static_cast<const _Gps__ros_msg_type *>(untyped_ros_message);
  // Field name: latitude
  {
    cdr << ros_message->latitude;
  }

  // Field name: longitude
  {
    cdr << ros_message->longitude;
  }

  // Field name: height
  {
    cdr << ros_message->height;
  }

  // Field name: heading_gps
  {
    cdr << ros_message->heading_gps;
  }

  // Field name: heading_track
  {
    cdr << ros_message->heading_track;
  }

  // Field name: speed
  {
    cdr << ros_message->speed;
  }

  // Field name: satellite
  {
    cdr << ros_message->satellite;
  }

  // Field name: heading
  {
    cdr << ros_message->heading;
  }

  return true;
}

static bool _Gps__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _Gps__ros_msg_type * ros_message = static_cast<_Gps__ros_msg_type *>(untyped_ros_message);
  // Field name: latitude
  {
    cdr >> ros_message->latitude;
  }

  // Field name: longitude
  {
    cdr >> ros_message->longitude;
  }

  // Field name: height
  {
    cdr >> ros_message->height;
  }

  // Field name: heading_gps
  {
    cdr >> ros_message->heading_gps;
  }

  // Field name: heading_track
  {
    cdr >> ros_message->heading_track;
  }

  // Field name: speed
  {
    cdr >> ros_message->speed;
  }

  // Field name: satellite
  {
    cdr >> ros_message->satellite;
  }

  // Field name: heading
  {
    cdr >> ros_message->heading;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rover_msgs
size_t get_serialized_size_rover_msgs__msg__Gps(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Gps__ros_msg_type * ros_message = static_cast<const _Gps__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name latitude
  {
    size_t item_size = sizeof(ros_message->latitude);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name longitude
  {
    size_t item_size = sizeof(ros_message->longitude);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name height
  {
    size_t item_size = sizeof(ros_message->height);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name heading_gps
  {
    size_t item_size = sizeof(ros_message->heading_gps);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name heading_track
  {
    size_t item_size = sizeof(ros_message->heading_track);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name speed
  {
    size_t item_size = sizeof(ros_message->speed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name satellite
  {
    size_t item_size = sizeof(ros_message->satellite);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name heading
  {
    size_t item_size = sizeof(ros_message->heading);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _Gps__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_rover_msgs__msg__Gps(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rover_msgs
size_t max_serialized_size_rover_msgs__msg__Gps(
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

  // member: latitude
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: longitude
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: height
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: heading_gps
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: heading_track
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: speed
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: satellite
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: heading
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = rover_msgs__msg__Gps;
    is_plain =
      (
      offsetof(DataType, heading) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _Gps__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_rover_msgs__msg__Gps(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_Gps = {
  "rover_msgs::msg",
  "Gps",
  _Gps__cdr_serialize,
  _Gps__cdr_deserialize,
  _Gps__get_serialized_size,
  _Gps__max_serialized_size
};

static rosidl_message_type_support_t _Gps__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_Gps,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rover_msgs, msg, Gps)() {
  return &_Gps__type_support;
}

#if defined(__cplusplus)
}
#endif
