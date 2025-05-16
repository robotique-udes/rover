// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rover_msgs:msg/LightControl.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__MSG__DETAIL__LIGHT_CONTROL__BUILDER_HPP_
#define ROVER_MSGS__MSG__DETAIL__LIGHT_CONTROL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rover_msgs/msg/detail/light_control__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rover_msgs
{

namespace msg
{

namespace builder
{

class Init_LightControl_enable
{
public:
  Init_LightControl_enable()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rover_msgs::msg::LightControl enable(::rover_msgs::msg::LightControl::_enable_type arg)
  {
    msg_.enable = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_msgs::msg::LightControl msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_msgs::msg::LightControl>()
{
  return rover_msgs::msg::builder::Init_LightControl_enable();
}

}  // namespace rover_msgs

#endif  // ROVER_MSGS__MSG__DETAIL__LIGHT_CONTROL__BUILDER_HPP_
