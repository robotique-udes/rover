// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rover_msgs:msg/Aruco.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__MSG__DETAIL__ARUCO__BUILDER_HPP_
#define ROVER_MSGS__MSG__DETAIL__ARUCO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rover_msgs/msg/detail/aruco__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rover_msgs
{

namespace msg
{

namespace builder
{

class Init_Aruco_id
{
public:
  explicit Init_Aruco_id(::rover_msgs::msg::Aruco & msg)
  : msg_(msg)
  {}
  ::rover_msgs::msg::Aruco id(::rover_msgs::msg::Aruco::_id_type arg)
  {
    msg_.id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_msgs::msg::Aruco msg_;
};

class Init_Aruco_valid
{
public:
  Init_Aruco_valid()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Aruco_id valid(::rover_msgs::msg::Aruco::_valid_type arg)
  {
    msg_.valid = std::move(arg);
    return Init_Aruco_id(msg_);
  }

private:
  ::rover_msgs::msg::Aruco msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_msgs::msg::Aruco>()
{
  return rover_msgs::msg::builder::Init_Aruco_valid();
}

}  // namespace rover_msgs

#endif  // ROVER_MSGS__MSG__DETAIL__ARUCO__BUILDER_HPP_
