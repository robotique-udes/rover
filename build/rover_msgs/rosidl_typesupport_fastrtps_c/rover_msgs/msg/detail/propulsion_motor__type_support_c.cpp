// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from rover_msgs:msg/PropulsionMotor.idl
// generated code does not contain a copyright notice
#include "rover_msgs/msg/detail/propulsion_motor__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rover_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "rover_msgs/msg/detail/propulsion_motor__struct.h"
#include "rover_msgs/msg/detail/propulsion_motor__functions.h"
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


using _PropulsionMotor__ros_msg_type = rover_msgs__msg__PropulsionMotor;

static bool _PropulsionMotor__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _PropulsionMotor__ros_msg_type * ros_message = static_cast<const _PropulsionMotor__ros_msg_type *>(untyped_ros_message);
  // Field name: enable
  {
    size_t size = 4;
    auto array_ptr = ros_message->enable;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: target_speed
  {
    size_t size = 4;
    auto array_ptr = ros_message->target_speed;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: current_speed
  {
    size_t size = 4;
    auto array_ptr = ros_message->current_speed;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: close_loop
  {
    size_t size = 4;
    auto array_ptr = ros_message->close_loop;
    cdr.serializeArray(array_ptr, size);
  }

  return true;
}

static bool _PropulsionMotor__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _PropulsionMotor__ros_msg_type * ros_message = static_cast<_PropulsionMotor__ros_msg_type *>(untyped_ros_message);
  // Field name: enable
  {
    size_t size = 4;
    auto array_ptr = ros_message->enable;
    for (size_t i = 0; i < size; ++i) {
      uint8_t tmp;
      cdr >> tmp;
      array_ptr[i] = tmp ? true : false;
    }
  }

  // Field name: target_speed
  {
    size_t size = 4;
    auto array_ptr = ros_message->target_speed;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: current_speed
  {
    size_t size = 4;
    auto array_ptr = ros_message->current_speed;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: close_loop
  {
    size_t size = 4;
    auto array_ptr = ros_message->close_loop;
    for (size_t i = 0; i < size; ++i) {
      uint8_t tmp;
      cdr >> tmp;
      array_ptr[i] = tmp ? true : false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rover_msgs
size_t get_serialized_size_rover_msgs__msg__PropulsionMotor(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _PropulsionMotor__ros_msg_type * ros_message = static_cast<const _PropulsionMotor__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name enable
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->enable;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name target_speed
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->target_speed;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name current_speed
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->current_speed;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name close_loop
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->close_loop;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _PropulsionMotor__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_rover_msgs__msg__PropulsionMotor(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rover_msgs
size_t max_serialized_size_rover_msgs__msg__PropulsionMotor(
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

  // member: enable
  {
    size_t array_size = 4;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: target_speed
  {
    size_t array_size = 4;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: current_speed
  {
    size_t array_size = 4;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: close_loop
  {
    size_t array_size = 4;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = rover_msgs__msg__PropulsionMotor;
    is_plain =
      (
      offsetof(DataType, close_loop) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _PropulsionMotor__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_rover_msgs__msg__PropulsionMotor(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_PropulsionMotor = {
  "rover_msgs::msg",
  "PropulsionMotor",
  _PropulsionMotor__cdr_serialize,
  _PropulsionMotor__cdr_deserialize,
  _PropulsionMotor__get_serialized_size,
  _PropulsionMotor__max_serialized_size
};

static rosidl_message_type_support_t _PropulsionMotor__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_PropulsionMotor,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rover_msgs, msg, PropulsionMotor)() {
  return &_PropulsionMotor__type_support;
}

#if defined(__cplusplus)
}
#endif
