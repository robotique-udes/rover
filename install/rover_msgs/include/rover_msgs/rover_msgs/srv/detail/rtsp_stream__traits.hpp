// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rover_msgs:srv/RtspStream.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__SRV__DETAIL__RTSP_STREAM__TRAITS_HPP_
#define ROVER_MSGS__SRV__DETAIL__RTSP_STREAM__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rover_msgs/srv/detail/rtsp_stream__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace rover_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const RtspStream_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: stream_id
  {
    out << "stream_id: ";
    rosidl_generator_traits::value_to_yaml(msg.stream_id, out);
    out << ", ";
  }

  // member: demand
  {
    out << "demand: ";
    rosidl_generator_traits::value_to_yaml(msg.demand, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RtspStream_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: stream_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stream_id: ";
    rosidl_generator_traits::value_to_yaml(msg.stream_id, out);
    out << "\n";
  }

  // member: demand
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "demand: ";
    rosidl_generator_traits::value_to_yaml(msg.demand, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RtspStream_Request & msg, bool use_flow_style = false)
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
  const rover_msgs::srv::RtspStream_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  rover_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rover_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const rover_msgs::srv::RtspStream_Request & msg)
{
  return rover_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<rover_msgs::srv::RtspStream_Request>()
{
  return "rover_msgs::srv::RtspStream_Request";
}

template<>
inline const char * name<rover_msgs::srv::RtspStream_Request>()
{
  return "rover_msgs/srv/RtspStream_Request";
}

template<>
struct has_fixed_size<rover_msgs::srv::RtspStream_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rover_msgs::srv::RtspStream_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rover_msgs::srv::RtspStream_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rover_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const RtspStream_Response & msg,
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
  const RtspStream_Response & msg,
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

inline std::string to_yaml(const RtspStream_Response & msg, bool use_flow_style = false)
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
  const rover_msgs::srv::RtspStream_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  rover_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rover_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const rover_msgs::srv::RtspStream_Response & msg)
{
  return rover_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<rover_msgs::srv::RtspStream_Response>()
{
  return "rover_msgs::srv::RtspStream_Response";
}

template<>
inline const char * name<rover_msgs::srv::RtspStream_Response>()
{
  return "rover_msgs/srv/RtspStream_Response";
}

template<>
struct has_fixed_size<rover_msgs::srv::RtspStream_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<rover_msgs::srv::RtspStream_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<rover_msgs::srv::RtspStream_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rover_msgs::srv::RtspStream>()
{
  return "rover_msgs::srv::RtspStream";
}

template<>
inline const char * name<rover_msgs::srv::RtspStream>()
{
  return "rover_msgs/srv/RtspStream";
}

template<>
struct has_fixed_size<rover_msgs::srv::RtspStream>
  : std::integral_constant<
    bool,
    has_fixed_size<rover_msgs::srv::RtspStream_Request>::value &&
    has_fixed_size<rover_msgs::srv::RtspStream_Response>::value
  >
{
};

template<>
struct has_bounded_size<rover_msgs::srv::RtspStream>
  : std::integral_constant<
    bool,
    has_bounded_size<rover_msgs::srv::RtspStream_Request>::value &&
    has_bounded_size<rover_msgs::srv::RtspStream_Response>::value
  >
{
};

template<>
struct is_service<rover_msgs::srv::RtspStream>
  : std::true_type
{
};

template<>
struct is_service_request<rover_msgs::srv::RtspStream_Request>
  : std::true_type
{
};

template<>
struct is_service_response<rover_msgs::srv::RtspStream_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ROVER_MSGS__SRV__DETAIL__RTSP_STREAM__TRAITS_HPP_
