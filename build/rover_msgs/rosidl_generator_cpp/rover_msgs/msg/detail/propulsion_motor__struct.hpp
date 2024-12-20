// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rover_msgs:msg/PropulsionMotor.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__MSG__DETAIL__PROPULSION_MOTOR__STRUCT_HPP_
#define ROVER_MSGS__MSG__DETAIL__PROPULSION_MOTOR__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__rover_msgs__msg__PropulsionMotor __attribute__((deprecated))
#else
# define DEPRECATED__rover_msgs__msg__PropulsionMotor __declspec(deprecated)
#endif

namespace rover_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PropulsionMotor_
{
  using Type = PropulsionMotor_<ContainerAllocator>;

  explicit PropulsionMotor_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<bool, 4>::iterator, bool>(this->enable.begin(), this->enable.end(), false);
      std::fill<typename std::array<float, 4>::iterator, float>(this->target_speed.begin(), this->target_speed.end(), 0.0f);
      std::fill<typename std::array<float, 4>::iterator, float>(this->current_speed.begin(), this->current_speed.end(), 0.0f);
      std::fill<typename std::array<bool, 4>::iterator, bool>(this->close_loop.begin(), this->close_loop.end(), false);
    }
  }

  explicit PropulsionMotor_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : enable(_alloc),
    target_speed(_alloc),
    current_speed(_alloc),
    close_loop(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<bool, 4>::iterator, bool>(this->enable.begin(), this->enable.end(), false);
      std::fill<typename std::array<float, 4>::iterator, float>(this->target_speed.begin(), this->target_speed.end(), 0.0f);
      std::fill<typename std::array<float, 4>::iterator, float>(this->current_speed.begin(), this->current_speed.end(), 0.0f);
      std::fill<typename std::array<bool, 4>::iterator, bool>(this->close_loop.begin(), this->close_loop.end(), false);
    }
  }

  // field types and members
  using _enable_type =
    std::array<bool, 4>;
  _enable_type enable;
  using _target_speed_type =
    std::array<float, 4>;
  _target_speed_type target_speed;
  using _current_speed_type =
    std::array<float, 4>;
  _current_speed_type current_speed;
  using _close_loop_type =
    std::array<bool, 4>;
  _close_loop_type close_loop;

  // setters for named parameter idiom
  Type & set__enable(
    const std::array<bool, 4> & _arg)
  {
    this->enable = _arg;
    return *this;
  }
  Type & set__target_speed(
    const std::array<float, 4> & _arg)
  {
    this->target_speed = _arg;
    return *this;
  }
  Type & set__current_speed(
    const std::array<float, 4> & _arg)
  {
    this->current_speed = _arg;
    return *this;
  }
  Type & set__close_loop(
    const std::array<bool, 4> & _arg)
  {
    this->close_loop = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t FRONT_LEFT =
    0u;
  static constexpr uint8_t FRONT_RIGHT =
    1u;
  static constexpr uint8_t REAR_LEFT =
    2u;
  static constexpr uint8_t REAR_RIGHT =
    3u;

  // pointer types
  using RawPtr =
    rover_msgs::msg::PropulsionMotor_<ContainerAllocator> *;
  using ConstRawPtr =
    const rover_msgs::msg::PropulsionMotor_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rover_msgs::msg::PropulsionMotor_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rover_msgs::msg::PropulsionMotor_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rover_msgs::msg::PropulsionMotor_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rover_msgs::msg::PropulsionMotor_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rover_msgs::msg::PropulsionMotor_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rover_msgs::msg::PropulsionMotor_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rover_msgs::msg::PropulsionMotor_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rover_msgs::msg::PropulsionMotor_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rover_msgs__msg__PropulsionMotor
    std::shared_ptr<rover_msgs::msg::PropulsionMotor_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rover_msgs__msg__PropulsionMotor
    std::shared_ptr<rover_msgs::msg::PropulsionMotor_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PropulsionMotor_ & other) const
  {
    if (this->enable != other.enable) {
      return false;
    }
    if (this->target_speed != other.target_speed) {
      return false;
    }
    if (this->current_speed != other.current_speed) {
      return false;
    }
    if (this->close_loop != other.close_loop) {
      return false;
    }
    return true;
  }
  bool operator!=(const PropulsionMotor_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PropulsionMotor_

// alias to use template instance with default allocator
using PropulsionMotor =
  rover_msgs::msg::PropulsionMotor_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t PropulsionMotor_<ContainerAllocator>::FRONT_LEFT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t PropulsionMotor_<ContainerAllocator>::FRONT_RIGHT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t PropulsionMotor_<ContainerAllocator>::REAR_LEFT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t PropulsionMotor_<ContainerAllocator>::REAR_RIGHT;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace rover_msgs

#endif  // ROVER_MSGS__MSG__DETAIL__PROPULSION_MOTOR__STRUCT_HPP_
