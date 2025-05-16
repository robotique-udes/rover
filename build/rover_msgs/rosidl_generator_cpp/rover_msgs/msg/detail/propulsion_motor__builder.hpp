// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rover_msgs:msg/PropulsionMotor.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__MSG__DETAIL__PROPULSION_MOTOR__BUILDER_HPP_
#define ROVER_MSGS__MSG__DETAIL__PROPULSION_MOTOR__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rover_msgs/msg/detail/propulsion_motor__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rover_msgs
{

namespace msg
{

namespace builder
{

class Init_PropulsionMotor_close_loop
{
public:
  explicit Init_PropulsionMotor_close_loop(::rover_msgs::msg::PropulsionMotor & msg)
  : msg_(msg)
  {}
  ::rover_msgs::msg::PropulsionMotor close_loop(::rover_msgs::msg::PropulsionMotor::_close_loop_type arg)
  {
    msg_.close_loop = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_msgs::msg::PropulsionMotor msg_;
};

class Init_PropulsionMotor_current_speed
{
public:
  explicit Init_PropulsionMotor_current_speed(::rover_msgs::msg::PropulsionMotor & msg)
  : msg_(msg)
  {}
  Init_PropulsionMotor_close_loop current_speed(::rover_msgs::msg::PropulsionMotor::_current_speed_type arg)
  {
    msg_.current_speed = std::move(arg);
    return Init_PropulsionMotor_close_loop(msg_);
  }

private:
  ::rover_msgs::msg::PropulsionMotor msg_;
};

class Init_PropulsionMotor_target_speed
{
public:
  explicit Init_PropulsionMotor_target_speed(::rover_msgs::msg::PropulsionMotor & msg)
  : msg_(msg)
  {}
  Init_PropulsionMotor_current_speed target_speed(::rover_msgs::msg::PropulsionMotor::_target_speed_type arg)
  {
    msg_.target_speed = std::move(arg);
    return Init_PropulsionMotor_current_speed(msg_);
  }

private:
  ::rover_msgs::msg::PropulsionMotor msg_;
};

class Init_PropulsionMotor_enable
{
public:
  Init_PropulsionMotor_enable()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PropulsionMotor_target_speed enable(::rover_msgs::msg::PropulsionMotor::_enable_type arg)
  {
    msg_.enable = std::move(arg);
    return Init_PropulsionMotor_target_speed(msg_);
  }

private:
  ::rover_msgs::msg::PropulsionMotor msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_msgs::msg::PropulsionMotor>()
{
  return rover_msgs::msg::builder::Init_PropulsionMotor_enable();
}

}  // namespace rover_msgs

#endif  // ROVER_MSGS__MSG__DETAIL__PROPULSION_MOTOR__BUILDER_HPP_
