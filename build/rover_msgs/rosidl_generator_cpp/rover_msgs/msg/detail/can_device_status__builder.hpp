// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rover_msgs:msg/CanDeviceStatus.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__MSG__DETAIL__CAN_DEVICE_STATUS__BUILDER_HPP_
#define ROVER_MSGS__MSG__DETAIL__CAN_DEVICE_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rover_msgs/msg/detail/can_device_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rover_msgs
{

namespace msg
{

namespace builder
{

class Init_CanDeviceStatus_watchdog_ok
{
public:
  explicit Init_CanDeviceStatus_watchdog_ok(::rover_msgs::msg::CanDeviceStatus & msg)
  : msg_(msg)
  {}
  ::rover_msgs::msg::CanDeviceStatus watchdog_ok(::rover_msgs::msg::CanDeviceStatus::_watchdog_ok_type arg)
  {
    msg_.watchdog_ok = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_msgs::msg::CanDeviceStatus msg_;
};

class Init_CanDeviceStatus_error_state
{
public:
  explicit Init_CanDeviceStatus_error_state(::rover_msgs::msg::CanDeviceStatus & msg)
  : msg_(msg)
  {}
  Init_CanDeviceStatus_watchdog_ok error_state(::rover_msgs::msg::CanDeviceStatus::_error_state_type arg)
  {
    msg_.error_state = std::move(arg);
    return Init_CanDeviceStatus_watchdog_ok(msg_);
  }

private:
  ::rover_msgs::msg::CanDeviceStatus msg_;
};

class Init_CanDeviceStatus_id
{
public:
  Init_CanDeviceStatus_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CanDeviceStatus_error_state id(::rover_msgs::msg::CanDeviceStatus::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_CanDeviceStatus_error_state(msg_);
  }

private:
  ::rover_msgs::msg::CanDeviceStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_msgs::msg::CanDeviceStatus>()
{
  return rover_msgs::msg::builder::Init_CanDeviceStatus_id();
}

}  // namespace rover_msgs

#endif  // ROVER_MSGS__MSG__DETAIL__CAN_DEVICE_STATUS__BUILDER_HPP_
