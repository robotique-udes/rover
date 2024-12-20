// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rover_msgs:msg/Compass.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__MSG__DETAIL__COMPASS__STRUCT_HPP_
#define ROVER_MSGS__MSG__DETAIL__COMPASS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__rover_msgs__msg__Compass __attribute__((deprecated))
#else
# define DEPRECATED__rover_msgs__msg__Compass __declspec(deprecated)
#endif

namespace rover_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Compass_
{
  using Type = Compass_<ContainerAllocator>;

  explicit Compass_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->heading = 0.0f;
      this->pitch = 0.0f;
    }
  }

  explicit Compass_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->heading = 0.0f;
      this->pitch = 0.0f;
    }
  }

  // field types and members
  using _heading_type =
    float;
  _heading_type heading;
  using _pitch_type =
    float;
  _pitch_type pitch;

  // setters for named parameter idiom
  Type & set__heading(
    const float & _arg)
  {
    this->heading = _arg;
    return *this;
  }
  Type & set__pitch(
    const float & _arg)
  {
    this->pitch = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rover_msgs::msg::Compass_<ContainerAllocator> *;
  using ConstRawPtr =
    const rover_msgs::msg::Compass_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rover_msgs::msg::Compass_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rover_msgs::msg::Compass_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rover_msgs::msg::Compass_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rover_msgs::msg::Compass_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rover_msgs::msg::Compass_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rover_msgs::msg::Compass_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rover_msgs::msg::Compass_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rover_msgs::msg::Compass_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rover_msgs__msg__Compass
    std::shared_ptr<rover_msgs::msg::Compass_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rover_msgs__msg__Compass
    std::shared_ptr<rover_msgs::msg::Compass_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Compass_ & other) const
  {
    if (this->heading != other.heading) {
      return false;
    }
    if (this->pitch != other.pitch) {
      return false;
    }
    return true;
  }
  bool operator!=(const Compass_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Compass_

// alias to use template instance with default allocator
using Compass =
  rover_msgs::msg::Compass_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rover_msgs

#endif  // ROVER_MSGS__MSG__DETAIL__COMPASS__STRUCT_HPP_
