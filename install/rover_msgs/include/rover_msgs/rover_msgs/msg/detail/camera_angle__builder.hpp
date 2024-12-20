// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rover_msgs:msg/CameraAngle.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__MSG__DETAIL__CAMERA_ANGLE__BUILDER_HPP_
#define ROVER_MSGS__MSG__DETAIL__CAMERA_ANGLE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rover_msgs/msg/detail/camera_angle__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rover_msgs
{

namespace msg
{

namespace builder
{

class Init_CameraAngle_angle
{
public:
  Init_CameraAngle_angle()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rover_msgs::msg::CameraAngle angle(::rover_msgs::msg::CameraAngle::_angle_type arg)
  {
    msg_.angle = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_msgs::msg::CameraAngle msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_msgs::msg::CameraAngle>()
{
  return rover_msgs::msg::builder::Init_CameraAngle_angle();
}

}  // namespace rover_msgs

#endif  // ROVER_MSGS__MSG__DETAIL__CAMERA_ANGLE__BUILDER_HPP_
