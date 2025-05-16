// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rover_msgs:srv/ScreenshotControl.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__SRV__DETAIL__SCREENSHOT_CONTROL__TRAITS_HPP_
#define ROVER_MSGS__SRV__DETAIL__SCREENSHOT_CONTROL__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rover_msgs/srv/detail/screenshot_control__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace rover_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const ScreenshotControl_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: start
  {
    out << "start: ";
    rosidl_generator_traits::value_to_yaml(msg.start, out);
    out << ", ";
  }

  // member: name
  {
    out << "name: ";
    rosidl_generator_traits::value_to_yaml(msg.name, out);
    out << ", ";
  }

  // member: ip_address
  {
    out << "ip_address: ";
    rosidl_generator_traits::value_to_yaml(msg.ip_address, out);
    out << ", ";
  }

  // member: metadata
  {
    out << "metadata: ";
    rosidl_generator_traits::value_to_yaml(msg.metadata, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ScreenshotControl_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: start
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "start: ";
    rosidl_generator_traits::value_to_yaml(msg.start, out);
    out << "\n";
  }

  // member: name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "name: ";
    rosidl_generator_traits::value_to_yaml(msg.name, out);
    out << "\n";
  }

  // member: ip_address
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ip_address: ";
    rosidl_generator_traits::value_to_yaml(msg.ip_address, out);
    out << "\n";
  }

  // member: metadata
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "metadata: ";
    rosidl_generator_traits::value_to_yaml(msg.metadata, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ScreenshotControl_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace rover_msgs

namespace rosidl_generator_traits
{

[[deprecated("use rover_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rover_msgs::srv::ScreenshotControl_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  rover_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rover_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const rover_msgs::srv::ScreenshotControl_Request & msg)
{
  return rover_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<rover_msgs::srv::ScreenshotControl_Request>()
{
  return "rover_msgs::srv::ScreenshotControl_Request";
}

template<>
inline const char * name<rover_msgs::srv::ScreenshotControl_Request>()
{
  return "rover_msgs/srv/ScreenshotControl_Request";
}

template<>
struct has_fixed_size<rover_msgs::srv::ScreenshotControl_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rover_msgs::srv::ScreenshotControl_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rover_msgs::srv::ScreenshotControl_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rover_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const ScreenshotControl_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: status_message
  {
    out << "status_message: ";
    rosidl_generator_traits::value_to_yaml(msg.status_message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ScreenshotControl_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: status_message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status_message: ";
    rosidl_generator_traits::value_to_yaml(msg.status_message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ScreenshotControl_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace rover_msgs

namespace rosidl_generator_traits
{

[[deprecated("use rover_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rover_msgs::srv::ScreenshotControl_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  rover_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rover_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const rover_msgs::srv::ScreenshotControl_Response & msg)
{
  return rover_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<rover_msgs::srv::ScreenshotControl_Response>()
{
  return "rover_msgs::srv::ScreenshotControl_Response";
}

template<>
inline const char * name<rover_msgs::srv::ScreenshotControl_Response>()
{
  return "rover_msgs/srv/ScreenshotControl_Response";
}

template<>
struct has_fixed_size<rover_msgs::srv::ScreenshotControl_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rover_msgs::srv::ScreenshotControl_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rover_msgs::srv::ScreenshotControl_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rover_msgs::srv::ScreenshotControl>()
{
  return "rover_msgs::srv::ScreenshotControl";
}

template<>
inline const char * name<rover_msgs::srv::ScreenshotControl>()
{
  return "rover_msgs/srv/ScreenshotControl";
}

template<>
struct has_fixed_size<rover_msgs::srv::ScreenshotControl>
  : std::integral_constant<
    bool,
    has_fixed_size<rover_msgs::srv::ScreenshotControl_Request>::value &&
    has_fixed_size<rover_msgs::srv::ScreenshotControl_Response>::value
  >
{
};

template<>
struct has_bounded_size<rover_msgs::srv::ScreenshotControl>
  : std::integral_constant<
    bool,
    has_bounded_size<rover_msgs::srv::ScreenshotControl_Request>::value &&
    has_bounded_size<rover_msgs::srv::ScreenshotControl_Response>::value
  >
{
};

template<>
struct is_service<rover_msgs::srv::ScreenshotControl>
  : std::true_type
{
};

template<>
struct is_service_request<rover_msgs::srv::ScreenshotControl_Request>
  : std::true_type
{
};

template<>
struct is_service_response<rover_msgs::srv::ScreenshotControl_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ROVER_MSGS__SRV__DETAIL__SCREENSHOT_CONTROL__TRAITS_HPP_
