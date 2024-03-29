// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rover_msgs:msg/CanDeviceStatus.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__MSG__DETAIL__CAN_DEVICE_STATUS__STRUCT_HPP_
#define ROVER_MSGS__MSG__DETAIL__CAN_DEVICE_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__rover_msgs__msg__CanDeviceStatus __attribute__((deprecated))
#else
# define DEPRECATED__rover_msgs__msg__CanDeviceStatus __declspec(deprecated)
#endif

namespace rover_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CanDeviceStatus_
{
  using Type = CanDeviceStatus_<ContainerAllocator>;

  explicit CanDeviceStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0;
      this->error_state = 0;
      this->watchdog_ok = false;
    }
  }

  explicit CanDeviceStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0;
      this->error_state = 0;
      this->watchdog_ok = false;
    }
  }

  // field types and members
  using _id_type =
    uint16_t;
  _id_type id;
  using _error_state_type =
    uint8_t;
  _error_state_type error_state;
  using _watchdog_ok_type =
    bool;
  _watchdog_ok_type watchdog_ok;

  // setters for named parameter idiom
  Type & set__id(
    const uint16_t & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__error_state(
    const uint8_t & _arg)
  {
    this->error_state = _arg;
    return *this;
  }
  Type & set__watchdog_ok(
    const bool & _arg)
  {
    this->watchdog_ok = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t STATUS_OK =
    0u;
  static constexpr uint8_t STATUS_WARNING =
    1u;
  static constexpr uint8_t STATUS_ERROR =
    2u;

  // pointer types
  using RawPtr =
    rover_msgs::msg::CanDeviceStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const rover_msgs::msg::CanDeviceStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rover_msgs::msg::CanDeviceStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rover_msgs::msg::CanDeviceStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rover_msgs::msg::CanDeviceStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rover_msgs::msg::CanDeviceStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rover_msgs::msg::CanDeviceStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rover_msgs::msg::CanDeviceStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rover_msgs::msg::CanDeviceStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rover_msgs::msg::CanDeviceStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rover_msgs__msg__CanDeviceStatus
    std::shared_ptr<rover_msgs::msg::CanDeviceStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rover_msgs__msg__CanDeviceStatus
    std::shared_ptr<rover_msgs::msg::CanDeviceStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CanDeviceStatus_ & other) const
  {
    if (this->id != other.id) {
      return false;
    }
    if (this->error_state != other.error_state) {
      return false;
    }
    if (this->watchdog_ok != other.watchdog_ok) {
      return false;
    }
    return true;
  }
  bool operator!=(const CanDeviceStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CanDeviceStatus_

// alias to use template instance with default allocator
using CanDeviceStatus =
  rover_msgs::msg::CanDeviceStatus_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t CanDeviceStatus_<ContainerAllocator>::STATUS_OK;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t CanDeviceStatus_<ContainerAllocator>::STATUS_WARNING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t CanDeviceStatus_<ContainerAllocator>::STATUS_ERROR;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace rover_msgs

#endif  // ROVER_MSGS__MSG__DETAIL__CAN_DEVICE_STATUS__STRUCT_HPP_
