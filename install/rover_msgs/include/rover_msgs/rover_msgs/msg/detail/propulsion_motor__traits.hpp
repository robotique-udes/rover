// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rover_msgs:msg/PropulsionMotor.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__MSG__DETAIL__PROPULSION_MOTOR__TRAITS_HPP_
#define ROVER_MSGS__MSG__DETAIL__PROPULSION_MOTOR__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rover_msgs/msg/detail/propulsion_motor__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace rover_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const PropulsionMotor & msg,
  std::ostream & out)
{
  out << "{";
  // member: enable
  {
    if (msg.enable.size() == 0) {
      out << "enable: []";
    } else {
      out << "enable: [";
      size_t pending_items = msg.enable.size();
      for (auto item : msg.enable) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: target_speed
  {
    if (msg.target_speed.size() == 0) {
      out << "target_speed: []";
    } else {
      out << "target_speed: [";
      size_t pending_items = msg.target_speed.size();
      for (auto item : msg.target_speed) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: current_speed
  {
    if (msg.current_speed.size() == 0) {
      out << "current_speed: []";
    } else {
      out << "current_speed: [";
      size_t pending_items = msg.current_speed.size();
      for (auto item : msg.current_speed) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: close_loop
  {
    if (msg.close_loop.size() == 0) {
      out << "close_loop: []";
    } else {
      out << "close_loop: [";
      size_t pending_items = msg.close_loop.size();
      for (auto item : msg.close_loop) {
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
  const PropulsionMotor & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: enable
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.enable.size() == 0) {
      out << "enable: []\n";
    } else {
      out << "enable:\n";
      for (auto item : msg.enable) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: target_speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.target_speed.size() == 0) {
      out << "target_speed: []\n";
    } else {
      out << "target_speed:\n";
      for (auto item : msg.target_speed) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: current_speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.current_speed.size() == 0) {
      out << "current_speed: []\n";
    } else {
      out << "current_speed:\n";
      for (auto item : msg.current_speed) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: close_loop
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.close_loop.size() == 0) {
      out << "close_loop: []\n";
    } else {
      out << "close_loop:\n";
      for (auto item : msg.close_loop) {
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

inline std::string to_yaml(const PropulsionMotor & msg, bool use_flow_style = false)
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
  const rover_msgs::msg::PropulsionMotor & msg,
  std::ostream & out, size_t indentation = 0)
{
  rover_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rover_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const rover_msgs::msg::PropulsionMotor & msg)
{
  return rover_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<rover_msgs::msg::PropulsionMotor>()
{
  return "rover_msgs::msg::PropulsionMotor";
}

template<>
inline const char * name<rover_msgs::msg::PropulsionMotor>()
{
  return "rover_msgs/msg/PropulsionMotor";
}

template<>
struct has_fixed_size<rover_msgs::msg::PropulsionMotor>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<rover_msgs::msg::PropulsionMotor>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<rover_msgs::msg::PropulsionMotor>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROVER_MSGS__MSG__DETAIL__PROPULSION_MOTOR__TRAITS_HPP_
