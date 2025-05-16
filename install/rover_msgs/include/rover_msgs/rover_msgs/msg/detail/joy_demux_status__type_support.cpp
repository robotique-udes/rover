// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from rover_msgs:msg/JoyDemuxStatus.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "rover_msgs/msg/detail/joy_demux_status__struct.hpp"
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

void JoyDemuxStatus_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) rover_msgs::msg::JoyDemuxStatus(_init);
}

void JoyDemuxStatus_fini_function(void * message_memory)
{
  auto typed_message = static_cast<rover_msgs::msg::JoyDemuxStatus *>(message_memory);
  typed_message->~JoyDemuxStatus();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember JoyDemuxStatus_message_member_array[2] = {
  {
    "controller_main_topic",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_msgs::msg::JoyDemuxStatus, controller_main_topic),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "controller_secondary_topic",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_msgs::msg::JoyDemuxStatus, controller_secondary_topic),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers JoyDemuxStatus_message_members = {
  "rover_msgs::msg",  // message namespace
  "JoyDemuxStatus",  // message name
  2,  // number of fields
  sizeof(rover_msgs::msg::JoyDemuxStatus),
  JoyDemuxStatus_message_member_array,  // message members
  JoyDemuxStatus_init_function,  // function to initialize message memory (memory has to be allocated)
  JoyDemuxStatus_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t JoyDemuxStatus_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &JoyDemuxStatus_message_members,
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
get_message_type_support_handle<rover_msgs::msg::JoyDemuxStatus>()
{
  return &::rover_msgs::msg::rosidl_typesupport_introspection_cpp::JoyDemuxStatus_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, rover_msgs, msg, JoyDemuxStatus)() {
  return &::rover_msgs::msg::rosidl_typesupport_introspection_cpp::JoyDemuxStatus_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
