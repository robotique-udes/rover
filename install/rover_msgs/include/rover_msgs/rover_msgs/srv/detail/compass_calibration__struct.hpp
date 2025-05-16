// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rover_msgs:srv/CompassCalibration.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__SRV__DETAIL__COMPASS_CALIBRATION__STRUCT_HPP_
#define ROVER_MSGS__SRV__DETAIL__COMPASS_CALIBRATION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__rover_msgs__srv__CompassCalibration_Request __attribute__((deprecated))
#else
# define DEPRECATED__rover_msgs__srv__CompassCalibration_Request __declspec(deprecated)
#endif

namespace rover_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct CompassCalibration_Request_
{
  using Type = CompassCalibration_Request_<ContainerAllocator>;

  explicit CompassCalibration_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->angle_offset = 0;
    }
  }

  explicit CompassCalibration_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->angle_offset = 0;
    }
  }

  // field types and members
  using _angle_offset_type =
    int16_t;
  _angle_offset_type angle_offset;

  // setters for named parameter idiom
  Type & set__angle_offset(
    const int16_t & _arg)
  {
    this->angle_offset = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rover_msgs::srv::CompassCalibration_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const rover_msgs::srv::CompassCalibration_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rover_msgs::srv::CompassCalibration_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rover_msgs::srv::CompassCalibration_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rover_msgs::srv::CompassCalibration_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rover_msgs::srv::CompassCalibration_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rover_msgs::srv::CompassCalibration_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rover_msgs::srv::CompassCalibration_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rover_msgs::srv::CompassCalibration_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rover_msgs::srv::CompassCalibration_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rover_msgs__srv__CompassCalibration_Request
    std::shared_ptr<rover_msgs::srv::CompassCalibration_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rover_msgs__srv__CompassCalibration_Request
    std::shared_ptr<rover_msgs::srv::CompassCalibration_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CompassCalibration_Request_ & other) const
  {
    if (this->angle_offset != other.angle_offset) {
      return false;
    }
    return true;
  }
  bool operator!=(const CompassCalibration_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CompassCalibration_Request_

// alias to use template instance with default allocator
using CompassCalibration_Request =
  rover_msgs::srv::CompassCalibration_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace rover_msgs


#ifndef _WIN32
# define DEPRECATED__rover_msgs__srv__CompassCalibration_Response __attribute__((deprecated))
#else
# define DEPRECATED__rover_msgs__srv__CompassCalibration_Response __declspec(deprecated)
#endif

namespace rover_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct CompassCalibration_Response_
{
  using Type = CompassCalibration_Response_<ContainerAllocator>;

  explicit CompassCalibration_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit CompassCalibration_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    rover_msgs::srv::CompassCalibration_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const rover_msgs::srv::CompassCalibration_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rover_msgs::srv::CompassCalibration_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rover_msgs::srv::CompassCalibration_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rover_msgs::srv::CompassCalibration_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rover_msgs::srv::CompassCalibration_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rover_msgs::srv::CompassCalibration_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rover_msgs::srv::CompassCalibration_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rover_msgs::srv::CompassCalibration_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rover_msgs::srv::CompassCalibration_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rover_msgs__srv__CompassCalibration_Response
    std::shared_ptr<rover_msgs::srv::CompassCalibration_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rover_msgs__srv__CompassCalibration_Response
    std::shared_ptr<rover_msgs::srv::CompassCalibration_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CompassCalibration_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const CompassCalibration_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CompassCalibration_Response_

// alias to use template instance with default allocator
using CompassCalibration_Response =
  rover_msgs::srv::CompassCalibration_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace rover_msgs

namespace rover_msgs
{

namespace srv
{

struct CompassCalibration
{
  using Request = rover_msgs::srv::CompassCalibration_Request;
  using Response = rover_msgs::srv::CompassCalibration_Response;
};

}  // namespace srv

}  // namespace rover_msgs

#endif  // ROVER_MSGS__SRV__DETAIL__COMPASS_CALIBRATION__STRUCT_HPP_
