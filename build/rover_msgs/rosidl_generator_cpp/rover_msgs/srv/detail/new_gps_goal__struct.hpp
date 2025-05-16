// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rover_msgs:srv/NewGpsGoal.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__SRV__DETAIL__NEW_GPS_GOAL__STRUCT_HPP_
#define ROVER_MSGS__SRV__DETAIL__NEW_GPS_GOAL__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'waypoints'
#include "rover_msgs/msg/detail/gps_position__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__rover_msgs__srv__NewGpsGoal_Request __attribute__((deprecated))
#else
# define DEPRECATED__rover_msgs__srv__NewGpsGoal_Request __declspec(deprecated)
#endif

namespace rover_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct NewGpsGoal_Request_
{
  using Type = NewGpsGoal_Request_<ContainerAllocator>;

  explicit NewGpsGoal_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->type = 0;
      this->index = 0;
    }
  }

  explicit NewGpsGoal_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->type = 0;
      this->index = 0;
    }
  }

  // field types and members
  using _type_type =
    uint8_t;
  _type_type type;
  using _index_type =
    uint8_t;
  _index_type index;
  using _waypoints_type =
    std::vector<rover_msgs::msg::GpsPosition_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<rover_msgs::msg::GpsPosition_<ContainerAllocator>>>;
  _waypoints_type waypoints;

  // setters for named parameter idiom
  Type & set__type(
    const uint8_t & _arg)
  {
    this->type = _arg;
    return *this;
  }
  Type & set__index(
    const uint8_t & _arg)
  {
    this->index = _arg;
    return *this;
  }
  Type & set__waypoints(
    const std::vector<rover_msgs::msg::GpsPosition_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<rover_msgs::msg::GpsPosition_<ContainerAllocator>>> & _arg)
  {
    this->waypoints = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t GET_ROUTE =
    0u;
  static constexpr uint8_t NEW_ROUTE =
    1u;
  static constexpr uint8_t NEW_GOAL_END_APPEND =
    2u;
  static constexpr uint8_t NEW_GOAL_END_OVERWRITE =
    3u;
  static constexpr uint8_t NEW_WAYPOINT_BEFORE_END =
    4u;
  static constexpr uint8_t NEW_WAYPOINT_INDEX_INSERT =
    5u;
  static constexpr uint8_t NEW_WAYPOINT_INDEX_REPLACE =
    6u;
  static constexpr uint8_t CLEAR_WAYPOINT_INDEX =
    7u;
  static constexpr uint8_t CLEAR_ROUTE =
    10u;

  // pointer types
  using RawPtr =
    rover_msgs::srv::NewGpsGoal_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const rover_msgs::srv::NewGpsGoal_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rover_msgs::srv::NewGpsGoal_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rover_msgs::srv::NewGpsGoal_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rover_msgs::srv::NewGpsGoal_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rover_msgs::srv::NewGpsGoal_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rover_msgs::srv::NewGpsGoal_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rover_msgs::srv::NewGpsGoal_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rover_msgs::srv::NewGpsGoal_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rover_msgs::srv::NewGpsGoal_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rover_msgs__srv__NewGpsGoal_Request
    std::shared_ptr<rover_msgs::srv::NewGpsGoal_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rover_msgs__srv__NewGpsGoal_Request
    std::shared_ptr<rover_msgs::srv::NewGpsGoal_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const NewGpsGoal_Request_ & other) const
  {
    if (this->type != other.type) {
      return false;
    }
    if (this->index != other.index) {
      return false;
    }
    if (this->waypoints != other.waypoints) {
      return false;
    }
    return true;
  }
  bool operator!=(const NewGpsGoal_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct NewGpsGoal_Request_

// alias to use template instance with default allocator
using NewGpsGoal_Request =
  rover_msgs::srv::NewGpsGoal_Request_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t NewGpsGoal_Request_<ContainerAllocator>::GET_ROUTE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t NewGpsGoal_Request_<ContainerAllocator>::NEW_ROUTE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t NewGpsGoal_Request_<ContainerAllocator>::NEW_GOAL_END_APPEND;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t NewGpsGoal_Request_<ContainerAllocator>::NEW_GOAL_END_OVERWRITE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t NewGpsGoal_Request_<ContainerAllocator>::NEW_WAYPOINT_BEFORE_END;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t NewGpsGoal_Request_<ContainerAllocator>::NEW_WAYPOINT_INDEX_INSERT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t NewGpsGoal_Request_<ContainerAllocator>::NEW_WAYPOINT_INDEX_REPLACE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t NewGpsGoal_Request_<ContainerAllocator>::CLEAR_WAYPOINT_INDEX;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t NewGpsGoal_Request_<ContainerAllocator>::CLEAR_ROUTE;
#endif  // __cplusplus < 201703L

}  // namespace srv

}  // namespace rover_msgs


// Include directives for member types
// Member 'route'
// already included above
// #include "rover_msgs/msg/detail/gps_position__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__rover_msgs__srv__NewGpsGoal_Response __attribute__((deprecated))
#else
# define DEPRECATED__rover_msgs__srv__NewGpsGoal_Response __declspec(deprecated)
#endif

namespace rover_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct NewGpsGoal_Response_
{
  using Type = NewGpsGoal_Response_<ContainerAllocator>;

  explicit NewGpsGoal_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->status = "";
    }
  }

  explicit NewGpsGoal_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : status(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->status = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _status_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _status_type status;
  using _route_type =
    std::vector<rover_msgs::msg::GpsPosition_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<rover_msgs::msg::GpsPosition_<ContainerAllocator>>>;
  _route_type route;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__status(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__route(
    const std::vector<rover_msgs::msg::GpsPosition_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<rover_msgs::msg::GpsPosition_<ContainerAllocator>>> & _arg)
  {
    this->route = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rover_msgs::srv::NewGpsGoal_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const rover_msgs::srv::NewGpsGoal_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rover_msgs::srv::NewGpsGoal_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rover_msgs::srv::NewGpsGoal_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rover_msgs::srv::NewGpsGoal_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rover_msgs::srv::NewGpsGoal_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rover_msgs::srv::NewGpsGoal_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rover_msgs::srv::NewGpsGoal_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rover_msgs::srv::NewGpsGoal_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rover_msgs::srv::NewGpsGoal_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rover_msgs__srv__NewGpsGoal_Response
    std::shared_ptr<rover_msgs::srv::NewGpsGoal_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rover_msgs__srv__NewGpsGoal_Response
    std::shared_ptr<rover_msgs::srv::NewGpsGoal_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const NewGpsGoal_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->status != other.status) {
      return false;
    }
    if (this->route != other.route) {
      return false;
    }
    return true;
  }
  bool operator!=(const NewGpsGoal_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct NewGpsGoal_Response_

// alias to use template instance with default allocator
using NewGpsGoal_Response =
  rover_msgs::srv::NewGpsGoal_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace rover_msgs

namespace rover_msgs
{

namespace srv
{

struct NewGpsGoal
{
  using Request = rover_msgs::srv::NewGpsGoal_Request;
  using Response = rover_msgs::srv::NewGpsGoal_Response;
};

}  // namespace srv

}  // namespace rover_msgs

#endif  // ROVER_MSGS__SRV__DETAIL__NEW_GPS_GOAL__STRUCT_HPP_
