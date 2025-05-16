// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rover_msgs:msg/Compass.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__MSG__DETAIL__COMPASS__BUILDER_HPP_
#define ROVER_MSGS__MSG__DETAIL__COMPASS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rover_msgs/msg/detail/compass__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rover_msgs
{

namespace msg
{

namespace builder
{

class Init_Compass_pitch
{
public:
  explicit Init_Compass_pitch(::rover_msgs::msg::Compass & msg)
  : msg_(msg)
  {}
  ::rover_msgs::msg::Compass pitch(::rover_msgs::msg::Compass::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_msgs::msg::Compass msg_;
};

class Init_Compass_heading
{
public:
  Init_Compass_heading()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Compass_pitch heading(::rover_msgs::msg::Compass::_heading_type arg)
  {
    msg_.heading = std::move(arg);
    return Init_Compass_pitch(msg_);
  }

private:
  ::rover_msgs::msg::Compass msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_msgs::msg::Compass>()
{
  return rover_msgs::msg::builder::Init_Compass_heading();
}

}  // namespace rover_msgs

#endif  // ROVER_MSGS__MSG__DETAIL__COMPASS__BUILDER_HPP_
