// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from rover_msgs:msg/PropulsionMotor.idl
// generated code does not contain a copyright notice
#include "rover_msgs/msg/detail/propulsion_motor__rosidl_typesupport_fastrtps_cpp.hpp"
#include "rover_msgs/msg/detail/propulsion_motor__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace rover_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rover_msgs
cdr_serialize(
  const rover_msgs::msg::PropulsionMotor & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: enable
  {
    cdr << ros_message.enable;
  }
  // Member: target_speed
  {
    cdr << ros_message.target_speed;
  }
  // Member: current_speed
  {
    cdr << ros_message.current_speed;
  }
  // Member: close_loop
  {
    cdr << ros_message.close_loop;
  }
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rover_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  rover_msgs::msg::PropulsionMotor & ros_message)
{
  // Member: enable
  {
    cdr >> ros_message.enable;
  }

  // Member: target_speed
  {
    cdr >> ros_message.target_speed;
  }

  // Member: current_speed
  {
    cdr >> ros_message.current_speed;
  }

  // Member: close_loop
  {
    cdr >> ros_message.close_loop;
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rover_msgs
get_serialized_size(
  const rover_msgs::msg::PropulsionMotor & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: enable
  {
    size_t array_size = 4;
    size_t item_size = sizeof(ros_message.enable[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: target_speed
  {
    size_t array_size = 4;
    size_t item_size = sizeof(ros_message.target_speed[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: current_speed
  {
    size_t array_size = 4;
    size_t item_size = sizeof(ros_message.current_speed[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: close_loop
  {
    size_t array_size = 4;
    size_t item_size = sizeof(ros_message.close_loop[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rover_msgs
max_serialized_size_PropulsionMotor(
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


  // Member: enable
  {
    size_t array_size = 4;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: target_speed
  {
    size_t array_size = 4;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: current_speed
  {
    size_t array_size = 4;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: close_loop
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
    using DataType = rover_msgs::msg::PropulsionMotor;
    is_plain =
      (
      offsetof(DataType, close_loop) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _PropulsionMotor__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const rover_msgs::msg::PropulsionMotor *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _PropulsionMotor__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<rover_msgs::msg::PropulsionMotor *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _PropulsionMotor__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const rover_msgs::msg::PropulsionMotor *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _PropulsionMotor__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_PropulsionMotor(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _PropulsionMotor__callbacks = {
  "rover_msgs::msg",
  "PropulsionMotor",
  _PropulsionMotor__cdr_serialize,
  _PropulsionMotor__cdr_deserialize,
  _PropulsionMotor__get_serialized_size,
  _PropulsionMotor__max_serialized_size
};

static rosidl_message_type_support_t _PropulsionMotor__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_PropulsionMotor__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace rover_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_rover_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<rover_msgs::msg::PropulsionMotor>()
{
  return &rover_msgs::msg::typesupport_fastrtps_cpp::_PropulsionMotor__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, rover_msgs, msg, PropulsionMotor)() {
  return &rover_msgs::msg::typesupport_fastrtps_cpp::_PropulsionMotor__handle;
}

#ifdef __cplusplus
}
#endif
