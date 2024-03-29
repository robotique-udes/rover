// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rover_msgs:msg/AntennaCmd.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__MSG__DETAIL__ANTENNA_CMD__STRUCT_HPP_
#define ROVER_MSGS__MSG__DETAIL__ANTENNA_CMD__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__rover_msgs__msg__AntennaCmd __attribute__((deprecated))
#else
# define DEPRECATED__rover_msgs__msg__AntennaCmd __declspec(deprecated)
#endif

namespace rover_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct AntennaCmd_
{
  using Type = AntennaCmd_<ContainerAllocator>;

  explicit AntennaCmd_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->enable = false;
      this->speed = 0.0f;
    }
  }

  explicit AntennaCmd_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->enable = false;
      this->speed = 0.0f;
    }
  }

  // field types and members
  using _enable_type =
    bool;
  _enable_type enable;
  using _speed_type =
    float;
  _speed_type speed;

  // setters for named parameter idiom
  Type & set__enable(
    const bool & _arg)
  {
    this->enable = _arg;
    return *this;
  }
  Type & set__speed(
    const float & _arg)
  {
    this->speed = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rover_msgs::msg::AntennaCmd_<ContainerAllocator> *;
  using ConstRawPtr =
    const rover_msgs::msg::AntennaCmd_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rover_msgs::msg::AntennaCmd_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rover_msgs::msg::AntennaCmd_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rover_msgs::msg::AntennaCmd_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rover_msgs::msg::AntennaCmd_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rover_msgs::msg::AntennaCmd_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rover_msgs::msg::AntennaCmd_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rover_msgs::msg::AntennaCmd_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rover_msgs::msg::AntennaCmd_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rover_msgs__msg__AntennaCmd
    std::shared_ptr<rover_msgs::msg::AntennaCmd_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rover_msgs__msg__AntennaCmd
    std::shared_ptr<rover_msgs::msg::AntennaCmd_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AntennaCmd_ & other) const
  {
    if (this->enable != other.enable) {
      return false;
    }
    if (this->speed != other.speed) {
      return false;
    }
    return true;
  }
  bool operator!=(const AntennaCmd_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AntennaCmd_

// alias to use template instance with default allocator
using AntennaCmd =
  rover_msgs::msg::AntennaCmd_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rover_msgs

#endif  // ROVER_MSGS__MSG__DETAIL__ANTENNA_CMD__STRUCT_HPP_
