// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rover_msgs:srv/JoyDemuxSetState.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__SRV__DETAIL__JOY_DEMUX_SET_STATE__TRAITS_HPP_
#define ROVER_MSGS__SRV__DETAIL__JOY_DEMUX_SET_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rover_msgs/srv/detail/joy_demux_set_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace rover_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const JoyDemuxSetState_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: controller_type
  {
    out << "controller_type: ";
    rosidl_generator_traits::value_to_yaml(msg.controller_type, out);
    out << ", ";
  }

  // member: destination
  {
    out << "destination: ";
    rosidl_generator_traits::value_to_yaml(msg.destination, out);
    out << ", ";
  }

  // member: force
  {
    out << "force: ";
    rosidl_generator_traits::value_to_yaml(msg.force, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const JoyDemuxSetState_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: controller_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "controller_type: ";
    rosidl_generator_traits::value_to_yaml(msg.controller_type, out);
    out << "\n";
  }

  // member: destination
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "destination: ";
    rosidl_generator_traits::value_to_yaml(msg.destination, out);
    out << "\n";
  }

  // member: force
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "force: ";
    rosidl_generator_traits::value_to_yaml(msg.force, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const JoyDemuxSetState_Request & msg, bool use_flow_style = false)
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
  const rover_msgs::srv::JoyDemuxSetState_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  rover_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rover_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const rover_msgs::srv::JoyDemuxSetState_Request & msg)
{
  return rover_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<rover_msgs::srv::JoyDemuxSetState_Request>()
{
  return "rover_msgs::srv::JoyDemuxSetState_Request";
}

template<>
inline const char * name<rover_msgs::srv::JoyDemuxSetState_Request>()
{
  return "rover_msgs/srv/JoyDemuxSetState_Request";
}

template<>
struct has_fixed_size<rover_msgs::srv::JoyDemuxSetState_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<rover_msgs::srv::JoyDemuxSetState_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<rover_msgs::srv::JoyDemuxSetState_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rover_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const JoyDemuxSetState_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const JoyDemuxSetState_Response & msg,
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
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const JoyDemuxSetState_Response & msg, bool use_flow_style = false)
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
  const rover_msgs::srv::JoyDemuxSetState_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  rover_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rover_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const rover_msgs::srv::JoyDemuxSetState_Response & msg)
{
  return rover_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<rover_msgs::srv::JoyDemuxSetState_Response>()
{
  return "rover_msgs::srv::JoyDemuxSetState_Response";
}

template<>
inline const char * name<rover_msgs::srv::JoyDemuxSetState_Response>()
{
  return "rover_msgs/srv/JoyDemuxSetState_Response";
}

template<>
struct has_fixed_size<rover_msgs::srv::JoyDemuxSetState_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<rover_msgs::srv::JoyDemuxSetState_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<rover_msgs::srv::JoyDemuxSetState_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rover_msgs::srv::JoyDemuxSetState>()
{
  return "rover_msgs::srv::JoyDemuxSetState";
}

template<>
inline const char * name<rover_msgs::srv::JoyDemuxSetState>()
{
  return "rover_msgs/srv/JoyDemuxSetState";
}

template<>
struct has_fixed_size<rover_msgs::srv::JoyDemuxSetState>
  : std::integral_constant<
    bool,
    has_fixed_size<rover_msgs::srv::JoyDemuxSetState_Request>::value &&
    has_fixed_size<rover_msgs::srv::JoyDemuxSetState_Response>::value
  >
{
};

template<>
struct has_bounded_size<rover_msgs::srv::JoyDemuxSetState>
  : std::integral_constant<
    bool,
    has_bounded_size<rover_msgs::srv::JoyDemuxSetState_Request>::value &&
    has_bounded_size<rover_msgs::srv::JoyDemuxSetState_Response>::value
  >
{
};

template<>
struct is_service<rover_msgs::srv::JoyDemuxSetState>
  : std::true_type
{
};

template<>
struct is_service_request<rover_msgs::srv::JoyDemuxSetState_Request>
  : std::true_type
{
};

template<>
struct is_service_response<rover_msgs::srv::JoyDemuxSetState_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ROVER_MSGS__SRV__DETAIL__JOY_DEMUX_SET_STATE__TRAITS_HPP_
