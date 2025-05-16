// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rover_msgs:msg/ArmMsg.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__MSG__DETAIL__ARM_MSG__BUILDER_HPP_
#define ROVER_MSGS__MSG__DETAIL__ARM_MSG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rover_msgs/msg/detail/arm_msg__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rover_msgs
{

namespace msg
{

namespace builder
{

class Init_ArmMsg_data
{
public:
  Init_ArmMsg_data()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rover_msgs::msg::ArmMsg data(::rover_msgs::msg::ArmMsg::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_msgs::msg::ArmMsg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_msgs::msg::ArmMsg>()
{
  return rover_msgs::msg::builder::Init_ArmMsg_data();
}

}  // namespace rover_msgs

#endif  // ROVER_MSGS__MSG__DETAIL__ARM_MSG__BUILDER_HPP_
