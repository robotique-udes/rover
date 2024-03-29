// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rover_msgs:msg/Joy.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__MSG__DETAIL__JOY__BUILDER_HPP_
#define ROVER_MSGS__MSG__DETAIL__JOY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rover_msgs/msg/detail/joy__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rover_msgs
{

namespace msg
{

namespace builder
{

class Init_Joy_joy_data
{
public:
  Init_Joy_joy_data()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rover_msgs::msg::Joy joy_data(::rover_msgs::msg::Joy::_joy_data_type arg)
  {
    msg_.joy_data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_msgs::msg::Joy msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_msgs::msg::Joy>()
{
  return rover_msgs::msg::builder::Init_Joy_joy_data();
}

}  // namespace rover_msgs

#endif  // ROVER_MSGS__MSG__DETAIL__JOY__BUILDER_HPP_
