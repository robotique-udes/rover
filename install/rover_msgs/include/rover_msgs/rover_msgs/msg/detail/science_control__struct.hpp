// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rover_msgs:msg/ScienceControl.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__MSG__DETAIL__SCIENCE_CONTROL__STRUCT_HPP_
#define ROVER_MSGS__MSG__DETAIL__SCIENCE_CONTROL__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__rover_msgs__msg__ScienceControl __attribute__((deprecated))
#else
# define DEPRECATED__rover_msgs__msg__ScienceControl __declspec(deprecated)
#endif

namespace rover_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ScienceControl_
{
  using Type = ScienceControl_<ContainerAllocator>;

  explicit ScienceControl_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->cmd = 0;
      this->current_sample = 0;
      this->dig = false;
    }
  }

  explicit ScienceControl_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->cmd = 0;
      this->current_sample = 0;
      this->dig = false;
    }
  }

  // field types and members
  using _cmd_type =
    int8_t;
  _cmd_type cmd;
  using _current_sample_type =
    int8_t;
  _current_sample_type current_sample;
  using _dig_type =
    bool;
  _dig_type dig;

  // setters for named parameter idiom
  Type & set__cmd(
    const int8_t & _arg)
  {
    this->cmd = _arg;
    return *this;
  }
  Type & set__current_sample(
    const int8_t & _arg)
  {
    this->current_sample = _arg;
    return *this;
  }
  Type & set__dig(
    const bool & _arg)
  {
    this->dig = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t DOWN =
    0u;
  static constexpr uint8_t UP =
    1u;
  static constexpr uint8_t E1 =
    0u;
  static constexpr uint8_t E2 =
    1u;
  static constexpr uint8_t E3 =
    2u;

  // pointer types
  using RawPtr =
    rover_msgs::msg::ScienceControl_<ContainerAllocator> *;
  using ConstRawPtr =
    const rover_msgs::msg::ScienceControl_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rover_msgs::msg::ScienceControl_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rover_msgs::msg::ScienceControl_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rover_msgs::msg::ScienceControl_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rover_msgs::msg::ScienceControl_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rover_msgs::msg::ScienceControl_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rover_msgs::msg::ScienceControl_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rover_msgs::msg::ScienceControl_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rover_msgs::msg::ScienceControl_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rover_msgs__msg__ScienceControl
    std::shared_ptr<rover_msgs::msg::ScienceControl_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rover_msgs__msg__ScienceControl
    std::shared_ptr<rover_msgs::msg::ScienceControl_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ScienceControl_ & other) const
  {
    if (this->cmd != other.cmd) {
      return false;
    }
    if (this->current_sample != other.current_sample) {
      return false;
    }
    if (this->dig != other.dig) {
      return false;
    }
    return true;
  }
  bool operator!=(const ScienceControl_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ScienceControl_

// alias to use template instance with default allocator
using ScienceControl =
  rover_msgs::msg::ScienceControl_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ScienceControl_<ContainerAllocator>::DOWN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ScienceControl_<ContainerAllocator>::UP;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ScienceControl_<ContainerAllocator>::E1;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ScienceControl_<ContainerAllocator>::E2;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ScienceControl_<ContainerAllocator>::E3;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace rover_msgs

#endif  // ROVER_MSGS__MSG__DETAIL__SCIENCE_CONTROL__STRUCT_HPP_
