// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rover_msgs:srv/JoyDemuxSetState.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__SRV__DETAIL__JOY_DEMUX_SET_STATE__STRUCT_HPP_
#define ROVER_MSGS__SRV__DETAIL__JOY_DEMUX_SET_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__rover_msgs__srv__JoyDemuxSetState_Request __attribute__((deprecated))
#else
# define DEPRECATED__rover_msgs__srv__JoyDemuxSetState_Request __declspec(deprecated)
#endif

namespace rover_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct JoyDemuxSetState_Request_
{
  using Type = JoyDemuxSetState_Request_<ContainerAllocator>;

  explicit JoyDemuxSetState_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->controller_type = 0;
      this->destination = 0;
      this->force = false;
    }
  }

  explicit JoyDemuxSetState_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->controller_type = 0;
      this->destination = 0;
      this->force = false;
    }
  }

  // field types and members
  using _controller_type_type =
    uint8_t;
  _controller_type_type controller_type;
  using _destination_type =
    uint8_t;
  _destination_type destination;
  using _force_type =
    bool;
  _force_type force;

  // setters for named parameter idiom
  Type & set__controller_type(
    const uint8_t & _arg)
  {
    this->controller_type = _arg;
    return *this;
  }
  Type & set__destination(
    const uint8_t & _arg)
  {
    this->destination = _arg;
    return *this;
  }
  Type & set__force(
    const bool & _arg)
  {
    this->force = _arg;
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
    rover_msgs::srv::JoyDemuxSetState_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const rover_msgs::srv::JoyDemuxSetState_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rover_msgs::srv::JoyDemuxSetState_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rover_msgs::srv::JoyDemuxSetState_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rover_msgs::srv::JoyDemuxSetState_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rover_msgs::srv::JoyDemuxSetState_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rover_msgs::srv::JoyDemuxSetState_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rover_msgs::srv::JoyDemuxSetState_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rover_msgs::srv::JoyDemuxSetState_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rover_msgs::srv::JoyDemuxSetState_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rover_msgs__srv__JoyDemuxSetState_Request
    std::shared_ptr<rover_msgs::srv::JoyDemuxSetState_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rover_msgs__srv__JoyDemuxSetState_Request
    std::shared_ptr<rover_msgs::srv::JoyDemuxSetState_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const JoyDemuxSetState_Request_ & other) const
  {
    if (this->controller_type != other.controller_type) {
      return false;
    }
    if (this->destination != other.destination) {
      return false;
    }
    if (this->force != other.force) {
      return false;
    }
    return true;
  }
  bool operator!=(const JoyDemuxSetState_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct JoyDemuxSetState_Request_

// alias to use template instance with default allocator
using JoyDemuxSetState_Request =
  rover_msgs::srv::JoyDemuxSetState_Request_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t JoyDemuxSetState_Request_<ContainerAllocator>::CONTROLLER_MAIN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t JoyDemuxSetState_Request_<ContainerAllocator>::CONTROLLER_SECONDARY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t JoyDemuxSetState_Request_<ContainerAllocator>::DEST_DRIVE_TRAIN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t JoyDemuxSetState_Request_<ContainerAllocator>::DEST_ARM;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t JoyDemuxSetState_Request_<ContainerAllocator>::DEST_ANTENNA;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t JoyDemuxSetState_Request_<ContainerAllocator>::DEST_NONE;
#endif  // __cplusplus < 201703L

}  // namespace srv

}  // namespace rover_msgs


#ifndef _WIN32
# define DEPRECATED__rover_msgs__srv__JoyDemuxSetState_Response __attribute__((deprecated))
#else
# define DEPRECATED__rover_msgs__srv__JoyDemuxSetState_Response __declspec(deprecated)
#endif

namespace rover_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct JoyDemuxSetState_Response_
{
  using Type = JoyDemuxSetState_Response_<ContainerAllocator>;

  explicit JoyDemuxSetState_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit JoyDemuxSetState_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rover_msgs::srv::JoyDemuxSetState_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const rover_msgs::srv::JoyDemuxSetState_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rover_msgs::srv::JoyDemuxSetState_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rover_msgs::srv::JoyDemuxSetState_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rover_msgs::srv::JoyDemuxSetState_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rover_msgs::srv::JoyDemuxSetState_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rover_msgs::srv::JoyDemuxSetState_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rover_msgs::srv::JoyDemuxSetState_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rover_msgs::srv::JoyDemuxSetState_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rover_msgs::srv::JoyDemuxSetState_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rover_msgs__srv__JoyDemuxSetState_Response
    std::shared_ptr<rover_msgs::srv::JoyDemuxSetState_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rover_msgs__srv__JoyDemuxSetState_Response
    std::shared_ptr<rover_msgs::srv::JoyDemuxSetState_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const JoyDemuxSetState_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const JoyDemuxSetState_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct JoyDemuxSetState_Response_

// alias to use template instance with default allocator
using JoyDemuxSetState_Response =
  rover_msgs::srv::JoyDemuxSetState_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace rover_msgs

namespace rover_msgs
{

namespace srv
{

struct JoyDemuxSetState
{
  using Request = rover_msgs::srv::JoyDemuxSetState_Request;
  using Response = rover_msgs::srv::JoyDemuxSetState_Response;
};

}  // namespace srv

}  // namespace rover_msgs

#endif  // ROVER_MSGS__SRV__DETAIL__JOY_DEMUX_SET_STATE__STRUCT_HPP_
