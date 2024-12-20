// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from rover_msgs:msg/LightControl.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "rover_msgs/msg/detail/light_control__rosidl_typesupport_introspection_c.h"
#include "rover_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rover_msgs/msg/detail/light_control__functions.h"
#include "rover_msgs/msg/detail/light_control__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void rover_msgs__msg__LightControl__rosidl_typesupport_introspection_c__LightControl_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rover_msgs__msg__LightControl__init(message_memory);
}

void rover_msgs__msg__LightControl__rosidl_typesupport_introspection_c__LightControl_fini_function(void * message_memory)
{
  rover_msgs__msg__LightControl__fini(message_memory);
}

size_t rover_msgs__msg__LightControl__rosidl_typesupport_introspection_c__size_function__LightControl__enable(
  const void * untyped_member)
{
  (void)untyped_member;
  return 2;
}

const void * rover_msgs__msg__LightControl__rosidl_typesupport_introspection_c__get_const_function__LightControl__enable(
  const void * untyped_member, size_t index)
{
  const bool * member =
    (const bool *)(untyped_member);
  return &member[index];
}

void * rover_msgs__msg__LightControl__rosidl_typesupport_introspection_c__get_function__LightControl__enable(
  void * untyped_member, size_t index)
{
  bool * member =
    (bool *)(untyped_member);
  return &member[index];
}

void rover_msgs__msg__LightControl__rosidl_typesupport_introspection_c__fetch_function__LightControl__enable(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const bool * item =
    ((const bool *)
    rover_msgs__msg__LightControl__rosidl_typesupport_introspection_c__get_const_function__LightControl__enable(untyped_member, index));
  bool * value =
    (bool *)(untyped_value);
  *value = *item;
}

void rover_msgs__msg__LightControl__rosidl_typesupport_introspection_c__assign_function__LightControl__enable(
  void * untyped_member, size_t index, const void * untyped_value)
{
  bool * item =
    ((bool *)
    rover_msgs__msg__LightControl__rosidl_typesupport_introspection_c__get_function__LightControl__enable(untyped_member, index));
  const bool * value =
    (const bool *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember rover_msgs__msg__LightControl__rosidl_typesupport_introspection_c__LightControl_message_member_array[1] = {
  {
    "enable",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    2,  // array size
    false,  // is upper bound
    offsetof(rover_msgs__msg__LightControl, enable),  // bytes offset in struct
    NULL,  // default value
    rover_msgs__msg__LightControl__rosidl_typesupport_introspection_c__size_function__LightControl__enable,  // size() function pointer
    rover_msgs__msg__LightControl__rosidl_typesupport_introspection_c__get_const_function__LightControl__enable,  // get_const(index) function pointer
    rover_msgs__msg__LightControl__rosidl_typesupport_introspection_c__get_function__LightControl__enable,  // get(index) function pointer
    rover_msgs__msg__LightControl__rosidl_typesupport_introspection_c__fetch_function__LightControl__enable,  // fetch(index, &value) function pointer
    rover_msgs__msg__LightControl__rosidl_typesupport_introspection_c__assign_function__LightControl__enable,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers rover_msgs__msg__LightControl__rosidl_typesupport_introspection_c__LightControl_message_members = {
  "rover_msgs__msg",  // message namespace
  "LightControl",  // message name
  1,  // number of fields
  sizeof(rover_msgs__msg__LightControl),
  rover_msgs__msg__LightControl__rosidl_typesupport_introspection_c__LightControl_message_member_array,  // message members
  rover_msgs__msg__LightControl__rosidl_typesupport_introspection_c__LightControl_init_function,  // function to initialize message memory (memory has to be allocated)
  rover_msgs__msg__LightControl__rosidl_typesupport_introspection_c__LightControl_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t rover_msgs__msg__LightControl__rosidl_typesupport_introspection_c__LightControl_message_type_support_handle = {
  0,
  &rover_msgs__msg__LightControl__rosidl_typesupport_introspection_c__LightControl_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rover_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rover_msgs, msg, LightControl)() {
  if (!rover_msgs__msg__LightControl__rosidl_typesupport_introspection_c__LightControl_message_type_support_handle.typesupport_identifier) {
    rover_msgs__msg__LightControl__rosidl_typesupport_introspection_c__LightControl_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &rover_msgs__msg__LightControl__rosidl_typesupport_introspection_c__LightControl_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
