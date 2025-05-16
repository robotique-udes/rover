// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rover_msgs:msg/ScienceControl.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__MSG__DETAIL__SCIENCE_CONTROL__BUILDER_HPP_
#define ROVER_MSGS__MSG__DETAIL__SCIENCE_CONTROL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rover_msgs/msg/detail/science_control__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rover_msgs
{

namespace msg
{

namespace builder
{

class Init_ScienceControl_dig
{
public:
  explicit Init_ScienceControl_dig(::rover_msgs::msg::ScienceControl & msg)
  : msg_(msg)
  {}
  ::rover_msgs::msg::ScienceControl dig(::rover_msgs::msg::ScienceControl::_dig_type arg)
  {
    msg_.dig = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_msgs::msg::ScienceControl msg_;
};

class Init_ScienceControl_current_sample
{
public:
  explicit Init_ScienceControl_current_sample(::rover_msgs::msg::ScienceControl & msg)
  : msg_(msg)
  {}
  Init_ScienceControl_dig current_sample(::rover_msgs::msg::ScienceControl::_current_sample_type arg)
  {
    msg_.current_sample = std::move(arg);
    return Init_ScienceControl_dig(msg_);
  }

private:
  ::rover_msgs::msg::ScienceControl msg_;
};

class Init_ScienceControl_cmd
{
public:
  Init_ScienceControl_cmd()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ScienceControl_current_sample cmd(::rover_msgs::msg::ScienceControl::_cmd_type arg)
  {
    msg_.cmd = std::move(arg);
    return Init_ScienceControl_current_sample(msg_);
  }

private:
  ::rover_msgs::msg::ScienceControl msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_msgs::msg::ScienceControl>()
{
  return rover_msgs::msg::builder::Init_ScienceControl_cmd();
}

}  // namespace rover_msgs

#endif  // ROVER_MSGS__MSG__DETAIL__SCIENCE_CONTROL__BUILDER_HPP_
