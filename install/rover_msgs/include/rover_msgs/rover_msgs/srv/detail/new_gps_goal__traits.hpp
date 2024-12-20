// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rover_msgs:srv/NewGpsGoal.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__SRV__DETAIL__NEW_GPS_GOAL__TRAITS_HPP_
#define ROVER_MSGS__SRV__DETAIL__NEW_GPS_GOAL__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rover_msgs/srv/detail/new_gps_goal__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'waypoints'
#include "rover_msgs/msg/detail/gps_position__traits.hpp"

namespace rover_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const NewGpsGoal_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: type
  {
    out << "type: ";
    rosidl_generator_traits::value_to_yaml(msg.type, out);
    out << ", ";
  }

  // member: index
  {
    out << "index: ";
    rosidl_generator_traits::value_to_yaml(msg.index, out);
    out << ", ";
  }

  // member: waypoints
  {
    if (msg.waypoints.size() == 0) {
      out << "waypoints: []";
    } else {
      out << "waypoints: [";
      size_t pending_items = msg.waypoints.size();
      for (auto item : msg.waypoints) {
        to_flow_style_yaml(item, out);
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
  const NewGpsGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "type: ";
    rosidl_generator_traits::value_to_yaml(msg.type, out);
    out << "\n";
  }

  // member: index
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "index: ";
    rosidl_generator_traits::value_to_yaml(msg.index, out);
    out << "\n";
  }

  // member: waypoints
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.waypoints.size() == 0) {
      out << "waypoints: []\n";
    } else {
      out << "waypoints:\n";
      for (auto item : msg.waypoints) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NewGpsGoal_Request & msg, bool use_flow_style = false)
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
  const rover_msgs::srv::NewGpsGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  rover_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rover_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const rover_msgs::srv::NewGpsGoal_Request & msg)
{
  return rover_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<rover_msgs::srv::NewGpsGoal_Request>()
{
  return "rover_msgs::srv::NewGpsGoal_Request";
}

template<>
inline const char * name<rover_msgs::srv::NewGpsGoal_Request>()
{
  return "rover_msgs/srv/NewGpsGoal_Request";
}

template<>
struct has_fixed_size<rover_msgs::srv::NewGpsGoal_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rover_msgs::srv::NewGpsGoal_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rover_msgs::srv::NewGpsGoal_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'route'
// already included above
// #include "rover_msgs/msg/detail/gps_position__traits.hpp"

namespace rover_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const NewGpsGoal_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << ", ";
  }

  // member: route
  {
    if (msg.route.size() == 0) {
      out << "route: []";
    } else {
      out << "route: [";
      size_t pending_items = msg.route.size();
      for (auto item : msg.route) {
        to_flow_style_yaml(item, out);
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
  const NewGpsGoal_Response & msg,
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

  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }

  // member: route
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.route.size() == 0) {
      out << "route: []\n";
    } else {
      out << "route:\n";
      for (auto item : msg.route) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NewGpsGoal_Response & msg, bool use_flow_style = false)
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
  const rover_msgs::srv::NewGpsGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  rover_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rover_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const rover_msgs::srv::NewGpsGoal_Response & msg)
{
  return rover_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<rover_msgs::srv::NewGpsGoal_Response>()
{
  return "rover_msgs::srv::NewGpsGoal_Response";
}

template<>
inline const char * name<rover_msgs::srv::NewGpsGoal_Response>()
{
  return "rover_msgs/srv/NewGpsGoal_Response";
}

template<>
struct has_fixed_size<rover_msgs::srv::NewGpsGoal_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rover_msgs::srv::NewGpsGoal_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rover_msgs::srv::NewGpsGoal_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rover_msgs::srv::NewGpsGoal>()
{
  return "rover_msgs::srv::NewGpsGoal";
}

template<>
inline const char * name<rover_msgs::srv::NewGpsGoal>()
{
  return "rover_msgs/srv/NewGpsGoal";
}

template<>
struct has_fixed_size<rover_msgs::srv::NewGpsGoal>
  : std::integral_constant<
    bool,
    has_fixed_size<rover_msgs::srv::NewGpsGoal_Request>::value &&
    has_fixed_size<rover_msgs::srv::NewGpsGoal_Response>::value
  >
{
};

template<>
struct has_bounded_size<rover_msgs::srv::NewGpsGoal>
  : std::integral_constant<
    bool,
    has_bounded_size<rover_msgs::srv::NewGpsGoal_Request>::value &&
    has_bounded_size<rover_msgs::srv::NewGpsGoal_Response>::value
  >
{
};

template<>
struct is_service<rover_msgs::srv::NewGpsGoal>
  : std::true_type
{
};

template<>
struct is_service_request<rover_msgs::srv::NewGpsGoal_Request>
  : std::true_type
{
};

template<>
struct is_service_response<rover_msgs::srv::NewGpsGoal_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ROVER_MSGS__SRV__DETAIL__NEW_GPS_GOAL__TRAITS_HPP_
