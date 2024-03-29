// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rover_msgs:srv/DriveTrainArbitration.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__SRV__DETAIL__DRIVE_TRAIN_ARBITRATION__STRUCT_HPP_
#define ROVER_MSGS__SRV__DETAIL__DRIVE_TRAIN_ARBITRATION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__rover_msgs__srv__DriveTrainArbitration_Request __attribute__((deprecated))
#else
# define DEPRECATED__rover_msgs__srv__DriveTrainArbitration_Request __declspec(deprecated)
#endif

namespace rover_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct DriveTrainArbitration_Request_
{
  using Type = DriveTrainArbitration_Request_<ContainerAllocator>;

  explicit DriveTrainArbitration_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->target_arbitration = 0;
    }
  }

  explicit DriveTrainArbitration_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->target_arbitration = 0;
    }
  }

  // field types and members
  using _target_arbitration_type =
    uint8_t;
  _target_arbitration_type target_arbitration;

  // setters for named parameter idiom
  Type & set__target_arbitration(
    const uint8_t & _arg)
  {
    this->target_arbitration = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t SECURITY =
    0u;
  static constexpr uint8_t TELEOP =
    1u;
  static constexpr uint8_t AUTONOMUS =
    2u;

  // pointer types
  using RawPtr =
    rover_msgs::srv::DriveTrainArbitration_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const rover_msgs::srv::DriveTrainArbitration_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rover_msgs::srv::DriveTrainArbitration_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rover_msgs::srv::DriveTrainArbitration_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rover_msgs::srv::DriveTrainArbitration_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rover_msgs::srv::DriveTrainArbitration_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rover_msgs::srv::DriveTrainArbitration_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rover_msgs::srv::DriveTrainArbitration_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rover_msgs::srv::DriveTrainArbitration_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rover_msgs::srv::DriveTrainArbitration_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rover_msgs__srv__DriveTrainArbitration_Request
    std::shared_ptr<rover_msgs::srv::DriveTrainArbitration_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rover_msgs__srv__DriveTrainArbitration_Request
    std::shared_ptr<rover_msgs::srv::DriveTrainArbitration_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DriveTrainArbitration_Request_ & other) const
  {
    if (this->target_arbitration != other.target_arbitration) {
      return false;
    }
    return true;
  }
  bool operator!=(const DriveTrainArbitration_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DriveTrainArbitration_Request_

// alias to use template instance with default allocator
using DriveTrainArbitration_Request =
  rover_msgs::srv::DriveTrainArbitration_Request_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DriveTrainArbitration_Request_<ContainerAllocator>::SECURITY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DriveTrainArbitration_Request_<ContainerAllocator>::TELEOP;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DriveTrainArbitration_Request_<ContainerAllocator>::AUTONOMUS;
#endif  // __cplusplus < 201703L

}  // namespace srv

}  // namespace rover_msgs


#ifndef _WIN32
# define DEPRECATED__rover_msgs__srv__DriveTrainArbitration_Response __attribute__((deprecated))
#else
# define DEPRECATED__rover_msgs__srv__DriveTrainArbitration_Response __declspec(deprecated)
#endif

namespace rover_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct DriveTrainArbitration_Response_
{
  using Type = DriveTrainArbitration_Response_<ContainerAllocator>;

  explicit DriveTrainArbitration_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->current_arbitration = 0;
    }
  }

  explicit DriveTrainArbitration_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->current_arbitration = 0;
    }
  }

  // field types and members
  using _current_arbitration_type =
    uint8_t;
  _current_arbitration_type current_arbitration;

  // setters for named parameter idiom
  Type & set__current_arbitration(
    const uint8_t & _arg)
  {
    this->current_arbitration = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rover_msgs::srv::DriveTrainArbitration_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const rover_msgs::srv::DriveTrainArbitration_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rover_msgs::srv::DriveTrainArbitration_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rover_msgs::srv::DriveTrainArbitration_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rover_msgs::srv::DriveTrainArbitration_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rover_msgs::srv::DriveTrainArbitration_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rover_msgs::srv::DriveTrainArbitration_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rover_msgs::srv::DriveTrainArbitration_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rover_msgs::srv::DriveTrainArbitration_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rover_msgs::srv::DriveTrainArbitration_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rover_msgs__srv__DriveTrainArbitration_Response
    std::shared_ptr<rover_msgs::srv::DriveTrainArbitration_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rover_msgs__srv__DriveTrainArbitration_Response
    std::shared_ptr<rover_msgs::srv::DriveTrainArbitration_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DriveTrainArbitration_Response_ & other) const
  {
    if (this->current_arbitration != other.current_arbitration) {
      return false;
    }
    return true;
  }
  bool operator!=(const DriveTrainArbitration_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DriveTrainArbitration_Response_

// alias to use template instance with default allocator
using DriveTrainArbitration_Response =
  rover_msgs::srv::DriveTrainArbitration_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace rover_msgs

namespace rover_msgs
{

namespace srv
{

struct DriveTrainArbitration
{
  using Request = rover_msgs::srv::DriveTrainArbitration_Request;
  using Response = rover_msgs::srv::DriveTrainArbitration_Response;
};

}  // namespace srv

}  // namespace rover_msgs

#endif  // ROVER_MSGS__SRV__DETAIL__DRIVE_TRAIN_ARBITRATION__STRUCT_HPP_
