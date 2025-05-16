// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rover_msgs:msg/Aruco.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__MSG__DETAIL__ARUCO__STRUCT_HPP_
#define ROVER_MSGS__MSG__DETAIL__ARUCO__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__rover_msgs__msg__Aruco __attribute__((deprecated))
#else
# define DEPRECATED__rover_msgs__msg__Aruco __declspec(deprecated)
#endif

namespace rover_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Aruco_
{
  using Type = Aruco_<ContainerAllocator>;

  explicit Aruco_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->valid = false;
    }
  }

  explicit Aruco_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->valid = false;
    }
  }

  // field types and members
  using _valid_type =
    bool;
  _valid_type valid;
  using _id_type =
    std::vector<uint16_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint16_t>>;
  _id_type id;

  // setters for named parameter idiom
  Type & set__valid(
    const bool & _arg)
  {
    this->valid = _arg;
    return *this;
  }
  Type & set__id(
    const std::vector<uint16_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint16_t>> & _arg)
  {
    this->id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rover_msgs::msg::Aruco_<ContainerAllocator> *;
  using ConstRawPtr =
    const rover_msgs::msg::Aruco_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rover_msgs::msg::Aruco_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rover_msgs::msg::Aruco_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rover_msgs::msg::Aruco_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rover_msgs::msg::Aruco_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rover_msgs::msg::Aruco_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rover_msgs::msg::Aruco_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rover_msgs::msg::Aruco_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rover_msgs::msg::Aruco_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rover_msgs__msg__Aruco
    std::shared_ptr<rover_msgs::msg::Aruco_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rover_msgs__msg__Aruco
    std::shared_ptr<rover_msgs::msg::Aruco_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Aruco_ & other) const
  {
    if (this->valid != other.valid) {
      return false;
    }
    if (this->id != other.id) {
      return false;
    }
    return true;
  }
  bool operator!=(const Aruco_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Aruco_

// alias to use template instance with default allocator
using Aruco =
  rover_msgs::msg::Aruco_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rover_msgs

#endif  // ROVER_MSGS__MSG__DETAIL__ARUCO__STRUCT_HPP_
