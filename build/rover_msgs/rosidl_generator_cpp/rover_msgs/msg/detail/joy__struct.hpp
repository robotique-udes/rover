// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rover_msgs:msg/Joy.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__MSG__DETAIL__JOY__STRUCT_HPP_
#define ROVER_MSGS__MSG__DETAIL__JOY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__rover_msgs__msg__Joy __attribute__((deprecated))
#else
# define DEPRECATED__rover_msgs__msg__Joy __declspec(deprecated)
#endif

namespace rover_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Joy_
{
  using Type = Joy_<ContainerAllocator>;

  explicit Joy_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<float, 20>::iterator, float>(this->joy_data.begin(), this->joy_data.end(), 0.0f);
    }
  }

  explicit Joy_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : joy_data(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<float, 20>::iterator, float>(this->joy_data.begin(), this->joy_data.end(), 0.0f);
    }
  }

  // field types and members
  using _joy_data_type =
    std::array<float, 20>;
  _joy_data_type joy_data;

  // setters for named parameter idiom
  Type & set__joy_data(
    const std::array<float, 20> & _arg)
  {
    this->joy_data = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t JOYSTICK_LEFT_FRONT =
    0u;
  static constexpr uint8_t JOYSTICK_LEFT_SIDE =
    1u;
  static constexpr uint8_t JOYSTICK_LEFT_PUSH =
    2u;
  static constexpr uint8_t JOYSTICK_RIGHT_FRONT =
    3u;
  static constexpr uint8_t JOYSTICK_RIGHT_SIDE =
    4u;
  static constexpr uint8_t JOYSTICK_RIGHT_PUSH =
    5u;
  static constexpr uint8_t CROSS_UP =
    6u;
  static constexpr uint8_t CROSS_DOWN =
    7u;
  static constexpr uint8_t CROSS_LEFT =
    8u;
  static constexpr uint8_t CROSS_RIGHT =
    9u;
  static constexpr uint8_t L1 =
    10u;
  static constexpr uint8_t L2 =
    11u;
  static constexpr uint8_t R1 =
    12u;
  static constexpr uint8_t R2 =
    13u;
  static constexpr uint8_t A =
    14u;
  static constexpr uint8_t B =
    15u;
  static constexpr uint8_t X =
    16u;
  static constexpr uint8_t Y =
    17u;
  static constexpr uint8_t EXT0 =
    18u;
  static constexpr uint8_t EXT1 =
    19u;
  static constexpr uint8_t EXT2 =
    20u;

  // pointer types
  using RawPtr =
    rover_msgs::msg::Joy_<ContainerAllocator> *;
  using ConstRawPtr =
    const rover_msgs::msg::Joy_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rover_msgs::msg::Joy_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rover_msgs::msg::Joy_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rover_msgs::msg::Joy_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rover_msgs::msg::Joy_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rover_msgs::msg::Joy_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rover_msgs::msg::Joy_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rover_msgs::msg::Joy_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rover_msgs::msg::Joy_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rover_msgs__msg__Joy
    std::shared_ptr<rover_msgs::msg::Joy_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rover_msgs__msg__Joy
    std::shared_ptr<rover_msgs::msg::Joy_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Joy_ & other) const
  {
    if (this->joy_data != other.joy_data) {
      return false;
    }
    return true;
  }
  bool operator!=(const Joy_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Joy_

// alias to use template instance with default allocator
using Joy =
  rover_msgs::msg::Joy_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Joy_<ContainerAllocator>::JOYSTICK_LEFT_FRONT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Joy_<ContainerAllocator>::JOYSTICK_LEFT_SIDE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Joy_<ContainerAllocator>::JOYSTICK_LEFT_PUSH;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Joy_<ContainerAllocator>::JOYSTICK_RIGHT_FRONT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Joy_<ContainerAllocator>::JOYSTICK_RIGHT_SIDE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Joy_<ContainerAllocator>::JOYSTICK_RIGHT_PUSH;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Joy_<ContainerAllocator>::CROSS_UP;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Joy_<ContainerAllocator>::CROSS_DOWN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Joy_<ContainerAllocator>::CROSS_LEFT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Joy_<ContainerAllocator>::CROSS_RIGHT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Joy_<ContainerAllocator>::L1;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Joy_<ContainerAllocator>::L2;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Joy_<ContainerAllocator>::R1;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Joy_<ContainerAllocator>::R2;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Joy_<ContainerAllocator>::A;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Joy_<ContainerAllocator>::B;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Joy_<ContainerAllocator>::X;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Joy_<ContainerAllocator>::Y;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Joy_<ContainerAllocator>::EXT0;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Joy_<ContainerAllocator>::EXT1;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Joy_<ContainerAllocator>::EXT2;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace rover_msgs

#endif  // ROVER_MSGS__MSG__DETAIL__JOY__STRUCT_HPP_
