// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rover_msgs:msg/GpsPosition.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__MSG__DETAIL__GPS_POSITION__STRUCT_HPP_
#define ROVER_MSGS__MSG__DETAIL__GPS_POSITION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__rover_msgs__msg__GpsPosition __attribute__((deprecated))
#else
# define DEPRECATED__rover_msgs__msg__GpsPosition __declspec(deprecated)
#endif

namespace rover_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GpsPosition_
{
  using Type = GpsPosition_<ContainerAllocator>;

  explicit GpsPosition_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->latitude = 0.0f;
      this->longitude = 0.0f;
    }
  }

  explicit GpsPosition_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->latitude = 0.0f;
      this->longitude = 0.0f;
    }
  }

  // field types and members
  using _latitude_type =
    float;
  _latitude_type latitude;
  using _longitude_type =
    float;
  _longitude_type longitude;

  // setters for named parameter idiom
  Type & set__latitude(
    const float & _arg)
  {
    this->latitude = _arg;
    return *this;
  }
  Type & set__longitude(
    const float & _arg)
  {
    this->longitude = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rover_msgs::msg::GpsPosition_<ContainerAllocator> *;
  using ConstRawPtr =
    const rover_msgs::msg::GpsPosition_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rover_msgs::msg::GpsPosition_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rover_msgs::msg::GpsPosition_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rover_msgs::msg::GpsPosition_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rover_msgs::msg::GpsPosition_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rover_msgs::msg::GpsPosition_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rover_msgs::msg::GpsPosition_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rover_msgs::msg::GpsPosition_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rover_msgs::msg::GpsPosition_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rover_msgs__msg__GpsPosition
    std::shared_ptr<rover_msgs::msg::GpsPosition_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rover_msgs__msg__GpsPosition
    std::shared_ptr<rover_msgs::msg::GpsPosition_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GpsPosition_ & other) const
  {
    if (this->latitude != other.latitude) {
      return false;
    }
    if (this->longitude != other.longitude) {
      return false;
    }
    return true;
  }
  bool operator!=(const GpsPosition_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GpsPosition_

// alias to use template instance with default allocator
using GpsPosition =
  rover_msgs::msg::GpsPosition_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rover_msgs

#endif  // ROVER_MSGS__MSG__DETAIL__GPS_POSITION__STRUCT_HPP_
