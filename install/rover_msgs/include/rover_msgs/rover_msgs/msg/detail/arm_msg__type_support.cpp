// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from rover_msgs:msg/ArmMsg.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "rover_msgs/msg/detail/arm_msg__struct.hpp"
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

void ArmMsg_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) rover_msgs::msg::ArmMsg(_init);
}

void ArmMsg_fini_function(void * message_memory)
{
  auto typed_message = static_cast<rover_msgs::msg::ArmMsg *>(message_memory);
  typed_message->~ArmMsg();
}

size_t size_function__ArmMsg__data(const void * untyped_member)
{
  (void)untyped_member;
  return 7;
}

const void * get_const_function__ArmMsg__data(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 7> *>(untyped_member);
  return &member[index];
}

void * get_function__ArmMsg__data(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 7> *>(untyped_member);
  return &member[index];
}

void fetch_function__ArmMsg__data(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__ArmMsg__data(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__ArmMsg__data(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__ArmMsg__data(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember ArmMsg_message_member_array[1] = {
  {
    "data",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    7,  // array size
    false,  // is upper bound
    offsetof(rover_msgs::msg::ArmMsg, data),  // bytes offset in struct
    nullptr,  // default value
    size_function__ArmMsg__data,  // size() function pointer
    get_const_function__ArmMsg__data,  // get_const(index) function pointer
    get_function__ArmMsg__data,  // get(index) function pointer
    fetch_function__ArmMsg__data,  // fetch(index, &value) function pointer
    assign_function__ArmMsg__data,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers ArmMsg_message_members = {
  "rover_msgs::msg",  // message namespace
  "ArmMsg",  // message name
  1,  // number of fields
  sizeof(rover_msgs::msg::ArmMsg),
  ArmMsg_message_member_array,  // message members
  ArmMsg_init_function,  // function to initialize message memory (memory has to be allocated)
  ArmMsg_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t ArmMsg_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &ArmMsg_message_members,
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
get_message_type_support_handle<rover_msgs::msg::ArmMsg>()
{
  return &::rover_msgs::msg::rosidl_typesupport_introspection_cpp::ArmMsg_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, rover_msgs, msg, ArmMsg)() {
  return &::rover_msgs::msg::rosidl_typesupport_introspection_cpp::ArmMsg_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
