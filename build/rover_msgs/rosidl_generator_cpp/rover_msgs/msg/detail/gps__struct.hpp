// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rover_msgs:msg/Gps.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__MSG__DETAIL__GPS__STRUCT_HPP_
#define ROVER_MSGS__MSG__DETAIL__GPS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__rover_msgs__msg__Gps __attribute__((deprecated))
#else
# define DEPRECATED__rover_msgs__msg__Gps __declspec(deprecated)
#endif

namespace rover_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Gps_
{
  using Type = Gps_<ContainerAllocator>;

  explicit Gps_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->latitude = 0.0f;
      this->longitude = 0.0f;
      this->height = 0.0f;
      this->heading_gps = 0.0f;
      this->heading_track = 0.0f;
      this->speed = 0.0f;
      this->satellite = 0;
      this->heading = 0.0f;
    }
  }

  explicit Gps_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->latitude = 0.0f;
      this->longitude = 0.0f;
      this->height = 0.0f;
      this->heading_gps = 0.0f;
      this->heading_track = 0.0f;
      this->speed = 0.0f;
      this->satellite = 0;
      this->heading = 0.0f;
    }
  }

  // field types and members
  using _latitude_type =
    float;
  _latitude_type latitude;
  using _longitude_type =
    float;
  _longitude_type longitude;
  using _height_type =
    float;
  _height_type height;
  using _heading_gps_type =
    float;
  _heading_gps_type heading_gps;
  using _heading_track_type =
    float;
  _heading_track_type heading_track;
  using _speed_type =
    float;
  _speed_type speed;
  using _satellite_type =
    uint8_t;
  _satellite_type satellite;
  using _heading_type =
    float;
  _heading_type heading;

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
  Type & set__height(
    const float & _arg)
  {
    this->height = _arg;
    return *this;
  }
  Type & set__heading_gps(
    const float & _arg)
  {
    this->heading_gps = _arg;
    return *this;
  }
  Type & set__heading_track(
    const float & _arg)
  {
    this->heading_track = _arg;
    return *this;
  }
  Type & set__speed(
    const float & _arg)
  {
    this->speed = _arg;
    return *this;
  }
  Type & set__satellite(
    const uint8_t & _arg)
  {
    this->satellite = _arg;
    return *this;
  }
  Type & set__heading(
    const float & _arg)
  {
    this->heading = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rover_msgs::msg::Gps_<ContainerAllocator> *;
  using ConstRawPtr =
    const rover_msgs::msg::Gps_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rover_msgs::msg::Gps_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rover_msgs::msg::Gps_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rover_msgs::msg::Gps_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rover_msgs::msg::Gps_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rover_msgs::msg::Gps_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rover_msgs::msg::Gps_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rover_msgs::msg::Gps_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rover_msgs::msg::Gps_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rover_msgs__msg__Gps
    std::shared_ptr<rover_msgs::msg::Gps_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rover_msgs__msg__Gps
    std::shared_ptr<rover_msgs::msg::Gps_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Gps_ & other) const
  {
    if (this->latitude != other.latitude) {
      return false;
    }
    if (this->longitude != other.longitude) {
      return false;
    }
    if (this->height != other.height) {
      return false;
    }
    if (this->heading_gps != other.heading_gps) {
      return false;
    }
    if (this->heading_track != other.heading_track) {
      return false;
    }
    if (this->speed != other.speed) {
      return false;
    }
    if (this->satellite != other.satellite) {
      return false;
    }
    if (this->heading != other.heading) {
      return false;
    }
    return true;
  }
  bool operator!=(const Gps_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Gps_

// alias to use template instance with default allocator
using Gps =
  rover_msgs::msg::Gps_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rover_msgs

#endif  // ROVER_MSGS__MSG__DETAIL__GPS__STRUCT_HPP_
