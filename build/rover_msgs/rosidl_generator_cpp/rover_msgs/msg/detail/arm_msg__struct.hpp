// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rover_msgs:msg/ArmMsg.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__MSG__DETAIL__ARM_MSG__STRUCT_HPP_
#define ROVER_MSGS__MSG__DETAIL__ARM_MSG__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__rover_msgs__msg__ArmMsg __attribute__((deprecated))
#else
# define DEPRECATED__rover_msgs__msg__ArmMsg __declspec(deprecated)
#endif

namespace rover_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ArmMsg_
{
  using Type = ArmMsg_<ContainerAllocator>;

  explicit ArmMsg_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<float, 7>::iterator, float>(this->data.begin(), this->data.end(), 0.0f);
    }
  }

  explicit ArmMsg_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : data(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<float, 7>::iterator, float>(this->data.begin(), this->data.end(), 0.0f);
    }
  }

  // field types and members
  using _data_type =
    std::array<float, 7>;
  _data_type data;

  // setters for named parameter idiom
  Type & set__data(
    const std::array<float, 7> & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t JL =
    0u;
  static constexpr uint8_t J0 =
    1u;
  static constexpr uint8_t J1 =
    2u;
  static constexpr uint8_t J2 =
    3u;
  static constexpr uint8_t GRIPPER_TILT =
    4u;
  static constexpr uint8_t GRIPPER_ROT =
    5u;
  static constexpr uint8_t GRIPPER_CLOSE =
    6u;

  // pointer types
  using RawPtr =
    rover_msgs::msg::ArmMsg_<ContainerAllocator> *;
  using ConstRawPtr =
    const rover_msgs::msg::ArmMsg_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rover_msgs::msg::ArmMsg_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rover_msgs::msg::ArmMsg_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rover_msgs::msg::ArmMsg_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rover_msgs::msg::ArmMsg_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rover_msgs::msg::ArmMsg_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rover_msgs::msg::ArmMsg_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rover_msgs::msg::ArmMsg_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rover_msgs::msg::ArmMsg_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rover_msgs__msg__ArmMsg
    std::shared_ptr<rover_msgs::msg::ArmMsg_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rover_msgs__msg__ArmMsg
    std::shared_ptr<rover_msgs::msg::ArmMsg_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ArmMsg_ & other) const
  {
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const ArmMsg_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ArmMsg_

// alias to use template instance with default allocator
using ArmMsg =
  rover_msgs::msg::ArmMsg_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ArmMsg_<ContainerAllocator>::JL;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ArmMsg_<ContainerAllocator>::J0;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ArmMsg_<ContainerAllocator>::J1;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ArmMsg_<ContainerAllocator>::J2;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ArmMsg_<ContainerAllocator>::GRIPPER_TILT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ArmMsg_<ContainerAllocator>::GRIPPER_ROT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ArmMsg_<ContainerAllocator>::GRIPPER_CLOSE;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace rover_msgs

#endif  // ROVER_MSGS__MSG__DETAIL__ARM_MSG__STRUCT_HPP_
