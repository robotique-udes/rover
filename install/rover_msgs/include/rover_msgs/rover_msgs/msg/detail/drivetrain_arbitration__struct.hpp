// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rover_msgs:msg/DrivetrainArbitration.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__MSG__DETAIL__DRIVETRAIN_ARBITRATION__STRUCT_HPP_
#define ROVER_MSGS__MSG__DETAIL__DRIVETRAIN_ARBITRATION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__rover_msgs__msg__DrivetrainArbitration __attribute__((deprecated))
#else
# define DEPRECATED__rover_msgs__msg__DrivetrainArbitration __declspec(deprecated)
#endif

namespace rover_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DrivetrainArbitration_
{
  using Type = DrivetrainArbitration_<ContainerAllocator>;

  explicit DrivetrainArbitration_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->arbitration = 0;
    }
  }

  explicit DrivetrainArbitration_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->arbitration = 0;
    }
  }

  // field types and members
  using _arbitration_type =
    uint8_t;
  _arbitration_type arbitration;

  // setters for named parameter idiom
  Type & set__arbitration(
    const uint8_t & _arg)
  {
    this->arbitration = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t NONE =
    0u;
  static constexpr uint8_t TELEOP =
    1u;
  static constexpr uint8_t AUTONOMUS =
    2u;

  // pointer types
  using RawPtr =
    rover_msgs::msg::DrivetrainArbitration_<ContainerAllocator> *;
  using ConstRawPtr =
    const rover_msgs::msg::DrivetrainArbitration_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rover_msgs::msg::DrivetrainArbitration_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rover_msgs::msg::DrivetrainArbitration_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rover_msgs::msg::DrivetrainArbitration_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rover_msgs::msg::DrivetrainArbitration_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rover_msgs::msg::DrivetrainArbitration_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rover_msgs::msg::DrivetrainArbitration_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rover_msgs::msg::DrivetrainArbitration_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rover_msgs::msg::DrivetrainArbitration_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rover_msgs__msg__DrivetrainArbitration
    std::shared_ptr<rover_msgs::msg::DrivetrainArbitration_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rover_msgs__msg__DrivetrainArbitration
    std::shared_ptr<rover_msgs::msg::DrivetrainArbitration_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DrivetrainArbitration_ & other) const
  {
    if (this->arbitration != other.arbitration) {
      return false;
    }
    return true;
  }
  bool operator!=(const DrivetrainArbitration_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DrivetrainArbitration_

// alias to use template instance with default allocator
using DrivetrainArbitration =
  rover_msgs::msg::DrivetrainArbitration_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DrivetrainArbitration_<ContainerAllocator>::NONE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DrivetrainArbitration_<ContainerAllocator>::TELEOP;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DrivetrainArbitration_<ContainerAllocator>::AUTONOMUS;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace rover_msgs

#endif  // ROVER_MSGS__MSG__DETAIL__DRIVETRAIN_ARBITRATION__STRUCT_HPP_
