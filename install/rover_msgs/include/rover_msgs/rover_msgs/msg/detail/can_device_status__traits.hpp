// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rover_msgs:msg/CanDeviceStatus.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__MSG__DETAIL__CAN_DEVICE_STATUS__TRAITS_HPP_
#define ROVER_MSGS__MSG__DETAIL__CAN_DEVICE_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rover_msgs/msg/detail/can_device_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace rover_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const CanDeviceStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: id
  {
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << ", ";
  }

  // member: error_state
  {
    out << "error_state: ";
    rosidl_generator_traits::value_to_yaml(msg.error_state, out);
    out << ", ";
  }

  // member: watchdog_ok
  {
    out << "watchdog_ok: ";
    rosidl_generator_traits::value_to_yaml(msg.watchdog_ok, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CanDeviceStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << "\n";
  }

  // member: error_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "error_state: ";
    rosidl_generator_traits::value_to_yaml(msg.error_state, out);
    out << "\n";
  }

  // member: watchdog_ok
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "watchdog_ok: ";
    rosidl_generator_traits::value_to_yaml(msg.watchdog_ok, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CanDeviceStatus & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace rover_msgs

namespace rosidl_generator_traits
{

[[deprecated("use rover_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rover_msgs::msg::CanDeviceStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  rover_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rover_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const rover_msgs::msg::CanDeviceStatus & msg)
{
  return rover_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<rover_msgs::msg::CanDeviceStatus>()
{
  return "rover_msgs::msg::CanDeviceStatus";
}

template<>
inline const char * name<rover_msgs::msg::CanDeviceStatus>()
{
  return "rover_msgs/msg/CanDeviceStatus";
}

template<>
struct has_fixed_size<rover_msgs::msg::CanDeviceStatus>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<rover_msgs::msg::CanDeviceStatus>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<rover_msgs::msg::CanDeviceStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROVER_MSGS__MSG__DETAIL__CAN_DEVICE_STATUS__TRAITS_HPP_
