// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rover_msgs:msg/ArmMsg.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__MSG__DETAIL__ARM_MSG__TRAITS_HPP_
#define ROVER_MSGS__MSG__DETAIL__ARM_MSG__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rover_msgs/msg/detail/arm_msg__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace rover_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const ArmMsg & msg,
  std::ostream & out)
{
  out << "{";
  // member: data
  {
    if (msg.data.size() == 0) {
      out << "data: []";
    } else {
      out << "data: [";
      size_t pending_items = msg.data.size();
      for (auto item : msg.data) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ArmMsg & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.data.size() == 0) {
      out << "data: []\n";
    } else {
      out << "data:\n";
      for (auto item : msg.data) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ArmMsg & msg, bool use_flow_style = false)
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
  const rover_msgs::msg::ArmMsg & msg,
  std::ostream & out, size_t indentation = 0)
{
  rover_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rover_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const rover_msgs::msg::ArmMsg & msg)
{
  return rover_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<rover_msgs::msg::ArmMsg>()
{
  return "rover_msgs::msg::ArmMsg";
}

template<>
inline const char * name<rover_msgs::msg::ArmMsg>()
{
  return "rover_msgs/msg/ArmMsg";
}

template<>
struct has_fixed_size<rover_msgs::msg::ArmMsg>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<rover_msgs::msg::ArmMsg>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<rover_msgs::msg::ArmMsg>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROVER_MSGS__MSG__DETAIL__ARM_MSG__TRAITS_HPP_
