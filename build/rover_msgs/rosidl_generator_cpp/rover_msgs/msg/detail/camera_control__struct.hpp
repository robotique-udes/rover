// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rover_msgs:msg/CameraControl.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__MSG__DETAIL__CAMERA_CONTROL__STRUCT_HPP_
#define ROVER_MSGS__MSG__DETAIL__CAMERA_CONTROL__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__rover_msgs__msg__CameraControl __attribute__((deprecated))
#else
# define DEPRECATED__rover_msgs__msg__CameraControl __declspec(deprecated)
#endif

namespace rover_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CameraControl_
{
  using Type = CameraControl_<ContainerAllocator>;

  explicit CameraControl_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<bool, 4>::iterator, bool>(this->enable.begin(), this->enable.end(), false);
    }
  }

  explicit CameraControl_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : enable(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<bool, 4>::iterator, bool>(this->enable.begin(), this->enable.end(), false);
    }
  }

  // field types and members
  using _enable_type =
    std::array<bool, 4>;
  _enable_type enable;

  // setters for named parameter idiom
  Type & set__enable(
    const std::array<bool, 4> & _arg)
  {
    this->enable = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t CAM_A2 =
    0u;
  static constexpr uint8_t CAM_R1M_1 =
    1u;
  static constexpr uint8_t CAM_R1M_2 =
    2u;
  static constexpr uint8_t CAM_R1M_3 =
    3u;

  // pointer types
  using RawPtr =
    rover_msgs::msg::CameraControl_<ContainerAllocator> *;
  using ConstRawPtr =
    const rover_msgs::msg::CameraControl_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rover_msgs::msg::CameraControl_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rover_msgs::msg::CameraControl_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rover_msgs::msg::CameraControl_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rover_msgs::msg::CameraControl_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rover_msgs::msg::CameraControl_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rover_msgs::msg::CameraControl_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rover_msgs::msg::CameraControl_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rover_msgs::msg::CameraControl_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rover_msgs__msg__CameraControl
    std::shared_ptr<rover_msgs::msg::CameraControl_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rover_msgs__msg__CameraControl
    std::shared_ptr<rover_msgs::msg::CameraControl_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CameraControl_ & other) const
  {
    if (this->enable != other.enable) {
      return false;
    }
    return true;
  }
  bool operator!=(const CameraControl_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CameraControl_

// alias to use template instance with default allocator
using CameraControl =
  rover_msgs::msg::CameraControl_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t CameraControl_<ContainerAllocator>::CAM_A2;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t CameraControl_<ContainerAllocator>::CAM_R1M_1;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t CameraControl_<ContainerAllocator>::CAM_R1M_2;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t CameraControl_<ContainerAllocator>::CAM_R1M_3;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace rover_msgs

#endif  // ROVER_MSGS__MSG__DETAIL__CAMERA_CONTROL__STRUCT_HPP_
