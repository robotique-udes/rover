// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from rover_msgs:msg/CameraControl.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "rover_msgs/msg/detail/camera_control__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace rover_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void CameraControl_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) rover_msgs::msg::CameraControl(_init);
}

void CameraControl_fini_function(void * message_memory)
{
  auto typed_message = static_cast<rover_msgs::msg::CameraControl *>(message_memory);
  typed_message->~CameraControl();
}

size_t size_function__CameraControl__enable(const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * get_const_function__CameraControl__enable(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<bool, 4> *>(untyped_member);
  return &member[index];
}

void * get_function__CameraControl__enable(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<bool, 4> *>(untyped_member);
  return &member[index];
}

void fetch_function__CameraControl__enable(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const bool *>(
    get_const_function__CameraControl__enable(untyped_member, index));
  auto & value = *reinterpret_cast<bool *>(untyped_value);
  value = item;
}

void assign_function__CameraControl__enable(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<bool *>(
    get_function__CameraControl__enable(untyped_member, index));
  const auto & value = *reinterpret_cast<const bool *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember CameraControl_message_member_array[1] = {
  {
    "enable",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(rover_msgs::msg::CameraControl, enable),  // bytes offset in struct
    nullptr,  // default value
    size_function__CameraControl__enable,  // size() function pointer
    get_const_function__CameraControl__enable,  // get_const(index) function pointer
    get_function__CameraControl__enable,  // get(index) function pointer
    fetch_function__CameraControl__enable,  // fetch(index, &value) function pointer
    assign_function__CameraControl__enable,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers CameraControl_message_members = {
  "rover_msgs::msg",  // message namespace
  "CameraControl",  // message name
  1,  // number of fields
  sizeof(rover_msgs::msg::CameraControl),
  CameraControl_message_member_array,  // message members
  CameraControl_init_function,  // function to initialize message memory (memory has to be allocated)
  CameraControl_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t CameraControl_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &CameraControl_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace rover_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<rover_msgs::msg::CameraControl>()
{
  return &::rover_msgs::msg::rosidl_typesupport_introspection_cpp::CameraControl_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, rover_msgs, msg, CameraControl)() {
  return &::rover_msgs::msg::rosidl_typesupport_introspection_cpp::CameraControl_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
