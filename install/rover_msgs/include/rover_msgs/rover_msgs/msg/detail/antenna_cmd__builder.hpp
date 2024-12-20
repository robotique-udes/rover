// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rover_msgs:msg/AntennaCmd.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__MSG__DETAIL__ANTENNA_CMD__BUILDER_HPP_
#define ROVER_MSGS__MSG__DETAIL__ANTENNA_CMD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rover_msgs/msg/detail/antenna_cmd__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rover_msgs
{

namespace msg
{

namespace builder
{

class Init_AntennaCmd_speed
{
public:
  explicit Init_AntennaCmd_speed(::rover_msgs::msg::AntennaCmd & msg)
  : msg_(msg)
  {}
  ::rover_msgs::msg::AntennaCmd speed(::rover_msgs::msg::AntennaCmd::_speed_type arg)
  {
    msg_.speed = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_msgs::msg::AntennaCmd msg_;
};

class Init_AntennaCmd_enable
{
public:
  Init_AntennaCmd_enable()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AntennaCmd_speed enable(::rover_msgs::msg::AntennaCmd::_enable_type arg)
  {
    msg_.enable = std::move(arg);
    return Init_AntennaCmd_speed(msg_);
  }

private:
  ::rover_msgs::msg::AntennaCmd msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_msgs::msg::AntennaCmd>()
{
  return rover_msgs::msg::builder::Init_AntennaCmd_enable();
}

}  // namespace rover_msgs

#endif  // ROVER_MSGS__MSG__DETAIL__ANTENNA_CMD__BUILDER_HPP_
