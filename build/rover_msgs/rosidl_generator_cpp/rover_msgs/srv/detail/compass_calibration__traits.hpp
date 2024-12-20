// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rover_msgs:srv/CompassCalibration.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__SRV__DETAIL__COMPASS_CALIBRATION__TRAITS_HPP_
#define ROVER_MSGS__SRV__DETAIL__COMPASS_CALIBRATION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rover_msgs/srv/detail/compass_calibration__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace rover_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const CompassCalibration_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: angle_offset
  {
    out << "angle_offset: ";
    rosidl_generator_traits::value_to_yaml(msg.angle_offset, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CompassCalibration_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: angle_offset
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "angle_offset: ";
    rosidl_generator_traits::value_to_yaml(msg.angle_offset, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CompassCalibration_Request & msg, bool use_flow_style = false)
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
  const rover_msgs::srv::CompassCalibration_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  rover_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rover_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const rover_msgs::srv::CompassCalibration_Request & msg)
{
  return rover_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<rover_msgs::srv::CompassCalibration_Request>()
{
  return "rover_msgs::srv::CompassCalibration_Request";
}

template<>
inline const char * name<rover_msgs::srv::CompassCalibration_Request>()
{
  return "rover_msgs/srv/CompassCalibration_Request";
}

template<>
struct has_fixed_size<rover_msgs::srv::CompassCalibration_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<rover_msgs::srv::CompassCalibration_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<rover_msgs::srv::CompassCalibration_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rover_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const CompassCalibration_Response & msg,
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
  const CompassCalibration_Response & msg,
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

inline std::string to_yaml(const CompassCalibration_Response & msg, bool use_flow_style = false)
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
  const rover_msgs::srv::CompassCalibration_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  rover_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rover_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const rover_msgs::srv::CompassCalibration_Response & msg)
{
  return rover_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<rover_msgs::srv::CompassCalibration_Response>()
{
  return "rover_msgs::srv::CompassCalibration_Response";
}

template<>
inline const char * name<rover_msgs::srv::CompassCalibration_Response>()
{
  return "rover_msgs/srv/CompassCalibration_Response";
}

template<>
struct has_fixed_size<rover_msgs::srv::CompassCalibration_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<rover_msgs::srv::CompassCalibration_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<rover_msgs::srv::CompassCalibration_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rover_msgs::srv::CompassCalibration>()
{
  return "rover_msgs::srv::CompassCalibration";
}

template<>
inline const char * name<rover_msgs::srv::CompassCalibration>()
{
  return "rover_msgs/srv/CompassCalibration";
}

template<>
struct has_fixed_size<rover_msgs::srv::CompassCalibration>
  : std::integral_constant<
    bool,
    has_fixed_size<rover_msgs::srv::CompassCalibration_Request>::value &&
    has_fixed_size<rover_msgs::srv::CompassCalibration_Response>::value
  >
{
};

template<>
struct has_bounded_size<rover_msgs::srv::CompassCalibration>
  : std::integral_constant<
    bool,
    has_bounded_size<rover_msgs::srv::CompassCalibration_Request>::value &&
    has_bounded_size<rover_msgs::srv::CompassCalibration_Response>::value
  >
{
};

template<>
struct is_service<rover_msgs::srv::CompassCalibration>
  : std::true_type
{
};

template<>
struct is_service_request<rover_msgs::srv::CompassCalibration_Request>
  : std::true_type
{
};

template<>
struct is_service_response<rover_msgs::srv::CompassCalibration_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ROVER_MSGS__SRV__DETAIL__COMPASS_CALIBRATION__TRAITS_HPP_
