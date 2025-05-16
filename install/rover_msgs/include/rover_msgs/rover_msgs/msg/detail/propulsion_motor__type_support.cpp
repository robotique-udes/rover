// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from rover_msgs:msg/PropulsionMotor.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "rover_msgs/msg/detail/propulsion_motor__struct.hpp"
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

void PropulsionMotor_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) rover_msgs::msg::PropulsionMotor(_init);
}

void PropulsionMotor_fini_function(void * message_memory)
{
  auto typed_message = static_cast<rover_msgs::msg::PropulsionMotor *>(message_memory);
  typed_message->~PropulsionMotor();
}

size_t size_function__PropulsionMotor__enable(const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * get_const_function__PropulsionMotor__enable(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<bool, 4> *>(untyped_member);
  return &member[index];
}

void * get_function__PropulsionMotor__enable(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<bool, 4> *>(untyped_member);
  return &member[index];
}

void fetch_function__PropulsionMotor__enable(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const bool *>(
    get_const_function__PropulsionMotor__enable(untyped_member, index));
  auto & value = *reinterpret_cast<bool *>(untyped_value);
  value = item;
}

void assign_function__PropulsionMotor__enable(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<bool *>(
    get_function__PropulsionMotor__enable(untyped_member, index));
  const auto & value = *reinterpret_cast<const bool *>(untyped_value);
  item = value;
}

size_t size_function__PropulsionMotor__target_speed(const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * get_const_function__PropulsionMotor__target_speed(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 4> *>(untyped_member);
  return &member[index];
}

void * get_function__PropulsionMotor__target_speed(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 4> *>(untyped_member);
  return &member[index];
}

void fetch_function__PropulsionMotor__target_speed(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__PropulsionMotor__target_speed(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__PropulsionMotor__target_speed(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__PropulsionMotor__target_speed(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

size_t size_function__PropulsionMotor__current_speed(const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * get_const_function__PropulsionMotor__current_speed(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 4> *>(untyped_member);
  return &member[index];
}

void * get_function__PropulsionMotor__current_speed(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 4> *>(untyped_member);
  return &member[index];
}

void fetch_function__PropulsionMotor__current_speed(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__PropulsionMotor__current_speed(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__PropulsionMotor__current_speed(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__PropulsionMotor__current_speed(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

size_t size_function__PropulsionMotor__close_loop(const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * get_const_function__PropulsionMotor__close_loop(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<bool, 4> *>(untyped_member);
  return &member[index];
}

void * get_function__PropulsionMotor__close_loop(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<bool, 4> *>(untyped_member);
  return &member[index];
}

void fetch_function__PropulsionMotor__close_loop(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const bool *>(
    get_const_function__PropulsionMotor__close_loop(untyped_member, index));
  auto & value = *reinterpret_cast<bool *>(untyped_value);
  value = item;
}

void assign_function__PropulsionMotor__close_loop(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<bool *>(
    get_function__PropulsionMotor__close_loop(untyped_member, index));
  const auto & value = *reinterpret_cast<const bool *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember PropulsionMotor_message_member_array[4] = {
  {
    "enable",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(rover_msgs::msg::PropulsionMotor, enable),  // bytes offset in struct
    nullptr,  // default value
    size_function__PropulsionMotor__enable,  // size() function pointer
    get_const_function__PropulsionMotor__enable,  // get_const(index) function pointer
    get_function__PropulsionMotor__enable,  // get(index) function pointer
    fetch_function__PropulsionMotor__enable,  // fetch(index, &value) function pointer
    assign_function__PropulsionMotor__enable,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "target_speed",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(rover_msgs::msg::PropulsionMotor, target_speed),  // bytes offset in struct
    nullptr,  // default value
    size_function__PropulsionMotor__target_speed,  // size() function pointer
    get_const_function__PropulsionMotor__target_speed,  // get_const(index) function pointer
    get_function__PropulsionMotor__target_speed,  // get(index) function pointer
    fetch_function__PropulsionMotor__target_speed,  // fetch(index, &value) function pointer
    assign_function__PropulsionMotor__target_speed,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "current_speed",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(rover_msgs::msg::PropulsionMotor, current_speed),  // bytes offset in struct
    nullptr,  // default value
    size_function__PropulsionMotor__current_speed,  // size() function pointer
    get_const_function__PropulsionMotor__current_speed,  // get_const(index) function pointer
    get_function__PropulsionMotor__current_speed,  // get(index) function pointer
    fetch_function__PropulsionMotor__current_speed,  // fetch(index, &value) function pointer
    assign_function__PropulsionMotor__current_speed,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "close_loop",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(rover_msgs::msg::PropulsionMotor, close_loop),  // bytes offset in struct
    nullptr,  // default value
    size_function__PropulsionMotor__close_loop,  // size() function pointer
    get_const_function__PropulsionMotor__close_loop,  // get_const(index) function pointer
    get_function__PropulsionMotor__close_loop,  // get(index) function pointer
    fetch_function__PropulsionMotor__close_loop,  // fetch(index, &value) function pointer
    assign_function__PropulsionMotor__close_loop,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers PropulsionMotor_message_members = {
  "rover_msgs::msg",  // message namespace
  "PropulsionMotor",  // message name
  4,  // number of fields
  sizeof(rover_msgs::msg::PropulsionMotor),
  PropulsionMotor_message_member_array,  // message members
  PropulsionMotor_init_function,  // function to initialize message memory (memory has to be allocated)
  PropulsionMotor_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t PropulsionMotor_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &PropulsionMotor_message_members,
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
get_message_type_support_handle<rover_msgs::msg::PropulsionMotor>()
{
  return &::rover_msgs::msg::rosidl_typesupport_introspection_cpp::PropulsionMotor_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, rover_msgs, msg, PropulsionMotor)() {
  return &::rover_msgs::msg::rosidl_typesupport_introspection_cpp::PropulsionMotor_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
