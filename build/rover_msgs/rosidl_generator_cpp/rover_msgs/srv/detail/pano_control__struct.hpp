// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rover_msgs:srv/PanoControl.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__SRV__DETAIL__PANO_CONTROL__STRUCT_HPP_
#define ROVER_MSGS__SRV__DETAIL__PANO_CONTROL__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__rover_msgs__srv__PanoControl_Request __attribute__((deprecated))
#else
# define DEPRECATED__rover_msgs__srv__PanoControl_Request __declspec(deprecated)
#endif

namespace rover_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct PanoControl_Request_
{
  using Type = PanoControl_Request_<ContainerAllocator>;

  explicit PanoControl_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->start = false;
      this->stop = false;
      this->photo = false;
      this->ip_address = "";
    }
  }

  explicit PanoControl_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : ip_address(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->start = false;
      this->stop = false;
      this->photo = false;
      this->ip_address = "";
    }
  }

  // field types and members
  using _start_type =
    bool;
  _start_type start;
  using _stop_type =
    bool;
  _stop_type stop;
  using _photo_type =
    bool;
  _photo_type photo;
  using _ip_address_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _ip_address_type ip_address;

  // setters for named parameter idiom
  Type & set__start(
    const bool & _arg)
  {
    this->start = _arg;
    return *this;
  }
  Type & set__stop(
    const bool & _arg)
  {
    this->stop = _arg;
    return *this;
  }
  Type & set__photo(
    const bool & _arg)
  {
    this->photo = _arg;
    return *this;
  }
  Type & set__ip_address(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->ip_address = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rover_msgs::srv::PanoControl_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const rover_msgs::srv::PanoControl_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rover_msgs::srv::PanoControl_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rover_msgs::srv::PanoControl_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rover_msgs::srv::PanoControl_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rover_msgs::srv::PanoControl_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rover_msgs::srv::PanoControl_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rover_msgs::srv::PanoControl_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rover_msgs::srv::PanoControl_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rover_msgs::srv::PanoControl_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rover_msgs__srv__PanoControl_Request
    std::shared_ptr<rover_msgs::srv::PanoControl_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rover_msgs__srv__PanoControl_Request
    std::shared_ptr<rover_msgs::srv::PanoControl_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PanoControl_Request_ & other) const
  {
    if (this->start != other.start) {
      return false;
    }
    if (this->stop != other.stop) {
      return false;
    }
    if (this->photo != other.photo) {
      return false;
    }
    if (this->ip_address != other.ip_address) {
      return false;
    }
    return true;
  }
  bool operator!=(const PanoControl_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PanoControl_Request_

// alias to use template instance with default allocator
using PanoControl_Request =
  rover_msgs::srv::PanoControl_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace rover_msgs


#ifndef _WIN32
# define DEPRECATED__rover_msgs__srv__PanoControl_Response __attribute__((deprecated))
#else
# define DEPRECATED__rover_msgs__srv__PanoControl_Response __declspec(deprecated)
#endif

namespace rover_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct PanoControl_Response_
{
  using Type = PanoControl_Response_<ContainerAllocator>;

  explicit PanoControl_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->status_message = "";
    }
  }

  explicit PanoControl_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : status_message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->status_message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _status_message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _status_message_type status_message;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__status_message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->status_message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rover_msgs::srv::PanoControl_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const rover_msgs::srv::PanoControl_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rover_msgs::srv::PanoControl_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rover_msgs::srv::PanoControl_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rover_msgs::srv::PanoControl_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rover_msgs::srv::PanoControl_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rover_msgs::srv::PanoControl_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rover_msgs::srv::PanoControl_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rover_msgs::srv::PanoControl_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rover_msgs::srv::PanoControl_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rover_msgs__srv__PanoControl_Response
    std::shared_ptr<rover_msgs::srv::PanoControl_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rover_msgs__srv__PanoControl_Response
    std::shared_ptr<rover_msgs::srv::PanoControl_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PanoControl_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->status_message != other.status_message) {
      return false;
    }
    return true;
  }
  bool operator!=(const PanoControl_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PanoControl_Response_

// alias to use template instance with default allocator
using PanoControl_Response =
  rover_msgs::srv::PanoControl_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace rover_msgs

namespace rover_msgs
{

namespace srv
{

struct PanoControl
{
  using Request = rover_msgs::srv::PanoControl_Request;
  using Response = rover_msgs::srv::PanoControl_Response;
};

}  // namespace srv

}  // namespace rover_msgs

#endif  // ROVER_MSGS__SRV__DETAIL__PANO_CONTROL__STRUCT_HPP_
