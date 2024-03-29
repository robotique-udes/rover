// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rover_msgs:msg/JoyDemuxStatus.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__MSG__DETAIL__JOY_DEMUX_STATUS__STRUCT_HPP_
#define ROVER_MSGS__MSG__DETAIL__JOY_DEMUX_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__rover_msgs__msg__JoyDemuxStatus __attribute__((deprecated))
#else
# define DEPRECATED__rover_msgs__msg__JoyDemuxStatus __declspec(deprecated)
#endif

namespace rover_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct JoyDemuxStatus_
{
  using Type = JoyDemuxStatus_<ContainerAllocator>;

  explicit JoyDemuxStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->controller_main_topic = 0;
      this->controller_secondary_topic = 0;
    }
  }

  explicit JoyDemuxStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->controller_main_topic = 0;
      this->controller_secondary_topic = 0;
    }
  }

  // field types and members
  using _controller_main_topic_type =
    uint8_t;
  _controller_main_topic_type controller_main_topic;
  using _controller_secondary_topic_type =
    uint8_t;
  _controller_secondary_topic_type controller_secondary_topic;

  // setters for named parameter idiom
  Type & set__controller_main_topic(
    const uint8_t & _arg)
  {
    this->controller_main_topic = _arg;
    return *this;
  }
  Type & set__controller_secondary_topic(
    const uint8_t & _arg)
  {
    this->controller_secondary_topic = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t CONTROLLER_MAIN =
    0u;
  static constexpr uint8_t CONTROLLER_SECONDARY =
    1u;
  static constexpr uint8_t DEST_DRIVE_TRAIN =
    0u;
  static constexpr uint8_t DEST_ARM =
    1u;
  static constexpr uint8_t DEST_ANTENNA =
    2u;
  static constexpr uint8_t DEST_NONE =
    3u;

  // pointer types
  using RawPtr =
    rover_msgs::msg::JoyDemuxStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const rover_msgs::msg::JoyDemuxStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rover_msgs::msg::JoyDemuxStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rover_msgs::msg::JoyDemuxStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rover_msgs::msg::JoyDemuxStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rover_msgs::msg::JoyDemuxStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rover_msgs::msg::JoyDemuxStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rover_msgs::msg::JoyDemuxStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rover_msgs::msg::JoyDemuxStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rover_msgs::msg::JoyDemuxStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rover_msgs__msg__JoyDemuxStatus
    std::shared_ptr<rover_msgs::msg::JoyDemuxStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rover_msgs__msg__JoyDemuxStatus
    std::shared_ptr<rover_msgs::msg::JoyDemuxStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const JoyDemuxStatus_ & other) const
  {
    if (this->controller_main_topic != other.controller_main_topic) {
      return false;
    }
    if (this->controller_secondary_topic != other.controller_secondary_topic) {
      return false;
    }
    return true;
  }
  bool operator!=(const JoyDemuxStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct JoyDemuxStatus_

// alias to use template instance with default allocator
using JoyDemuxStatus =
  rover_msgs::msg::JoyDemuxStatus_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t JoyDemuxStatus_<ContainerAllocator>::CONTROLLER_MAIN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t JoyDemuxStatus_<ContainerAllocator>::CONTROLLER_SECONDARY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t JoyDemuxStatus_<ContainerAllocator>::DEST_DRIVE_TRAIN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t JoyDemuxStatus_<ContainerAllocator>::DEST_ARM;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t JoyDemuxStatus_<ContainerAllocator>::DEST_ANTENNA;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t JoyDemuxStatus_<ContainerAllocator>::DEST_NONE;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace rover_msgs

#endif  // ROVER_MSGS__MSG__DETAIL__JOY_DEMUX_STATUS__STRUCT_HPP_
