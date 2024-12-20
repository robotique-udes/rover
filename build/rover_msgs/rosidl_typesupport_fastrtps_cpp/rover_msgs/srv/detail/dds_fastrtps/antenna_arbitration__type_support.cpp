// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from rover_msgs:srv/AntennaArbitration.idl
// generated code does not contain a copyright notice
#include "rover_msgs/srv/detail/antenna_arbitration__rosidl_typesupport_fastrtps_cpp.hpp"
#include "rover_msgs/srv/detail/antenna_arbitration__struct.hpp"

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

namespace srv
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rover_msgs
cdr_serialize(
  const rover_msgs::srv::AntennaArbitration_Request & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: target_arbitration
  cdr << ros_message.target_arbitration;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rover_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  rover_msgs::srv::AntennaArbitration_Request & ros_message)
{
  // Member: target_arbitration
  cdr >> ros_message.target_arbitration;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rover_msgs
get_serialized_size(
  const rover_msgs::srv::AntennaArbitration_Request & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: target_arbitration
  {
    size_t item_size = sizeof(ros_message.target_arbitration);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rover_msgs
max_serialized_size_AntennaArbitration_Request(
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


  // Member: target_arbitration
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
    using DataType = rover_msgs::srv::AntennaArbitration_Request;
    is_plain =
      (
      offsetof(DataType, target_arbitration) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _AntennaArbitration_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const rover_msgs::srv::AntennaArbitration_Request *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _AntennaArbitration_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<rover_msgs::srv::AntennaArbitration_Request *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _AntennaArbitration_Request__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const rover_msgs::srv::AntennaArbitration_Request *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _AntennaArbitration_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_AntennaArbitration_Request(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _AntennaArbitration_Request__callbacks = {
  "rover_msgs::srv",
  "AntennaArbitration_Request",
  _AntennaArbitration_Request__cdr_serialize,
  _AntennaArbitration_Request__cdr_deserialize,
  _AntennaArbitration_Request__get_serialized_size,
  _AntennaArbitration_Request__max_serialized_size
};

static rosidl_message_type_support_t _AntennaArbitration_Request__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_AntennaArbitration_Request__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace rover_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_rover_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<rover_msgs::srv::AntennaArbitration_Request>()
{
  return &rover_msgs::srv::typesupport_fastrtps_cpp::_AntennaArbitration_Request__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, rover_msgs, srv, AntennaArbitration_Request)() {
  return &rover_msgs::srv::typesupport_fastrtps_cpp::_AntennaArbitration_Request__handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include <limits>
// already included above
// #include <stdexcept>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
// already included above
// #include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace rover_msgs
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rover_msgs
cdr_serialize(
  const rover_msgs::srv::AntennaArbitration_Response & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: current_arbitration
  cdr << ros_message.current_arbitration;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rover_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  rover_msgs::srv::AntennaArbitration_Response & ros_message)
{
  // Member: current_arbitration
  cdr >> ros_message.current_arbitration;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rover_msgs
get_serialized_size(
  const rover_msgs::srv::AntennaArbitration_Response & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: current_arbitration
  {
    size_t item_size = sizeof(ros_message.current_arbitration);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rover_msgs
max_serialized_size_AntennaArbitration_Response(
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


  // Member: current_arbitration
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
    using DataType = rover_msgs::srv::AntennaArbitration_Response;
    is_plain =
      (
      offsetof(DataType, current_arbitration) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _AntennaArbitration_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const rover_msgs::srv::AntennaArbitration_Response *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _AntennaArbitration_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<rover_msgs::srv::AntennaArbitration_Response *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _AntennaArbitration_Response__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const rover_msgs::srv::AntennaArbitration_Response *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _AntennaArbitration_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_AntennaArbitration_Response(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _AntennaArbitration_Response__callbacks = {
  "rover_msgs::srv",
  "AntennaArbitration_Response",
  _AntennaArbitration_Response__cdr_serialize,
  _AntennaArbitration_Response__cdr_deserialize,
  _AntennaArbitration_Response__get_serialized_size,
  _AntennaArbitration_Response__max_serialized_size
};

static rosidl_message_type_support_t _AntennaArbitration_Response__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_AntennaArbitration_Response__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace rover_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_rover_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<rover_msgs::srv::AntennaArbitration_Response>()
{
  return &rover_msgs::srv::typesupport_fastrtps_cpp::_AntennaArbitration_Response__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, rover_msgs, srv, AntennaArbitration_Response)() {
  return &rover_msgs::srv::typesupport_fastrtps_cpp::_AntennaArbitration_Response__handle;
}

#ifdef __cplusplus
}
#endif

#include "rmw/error_handling.h"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/service_type_support_decl.hpp"

namespace rover_msgs
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

static service_type_support_callbacks_t _AntennaArbitration__callbacks = {
  "rover_msgs::srv",
  "AntennaArbitration",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, rover_msgs, srv, AntennaArbitration_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, rover_msgs, srv, AntennaArbitration_Response)(),
};

static rosidl_service_type_support_t _AntennaArbitration__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_AntennaArbitration__callbacks,
  get_service_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace rover_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_rover_msgs
const rosidl_service_type_support_t *
get_service_type_support_handle<rover_msgs::srv::AntennaArbitration>()
{
  return &rover_msgs::srv::typesupport_fastrtps_cpp::_AntennaArbitration__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, rover_msgs, srv, AntennaArbitration)() {
  return &rover_msgs::srv::typesupport_fastrtps_cpp::_AntennaArbitration__handle;
}

#ifdef __cplusplus
}
#endif
