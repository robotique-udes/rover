// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rover_msgs:msg/DrivetrainArbitration.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__MSG__DETAIL__DRIVETRAIN_ARBITRATION__BUILDER_HPP_
#define ROVER_MSGS__MSG__DETAIL__DRIVETRAIN_ARBITRATION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rover_msgs/msg/detail/drivetrain_arbitration__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rover_msgs
{

namespace msg
{

namespace builder
{

class Init_DrivetrainArbitration_arbitration
{
public:
  Init_DrivetrainArbitration_arbitration()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rover_msgs::msg::DrivetrainArbitration arbitration(::rover_msgs::msg::DrivetrainArbitration::_arbitration_type arg)
  {
    msg_.arbitration = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_msgs::msg::DrivetrainArbitration msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_msgs::msg::DrivetrainArbitration>()
{
  return rover_msgs::msg::builder::Init_DrivetrainArbitration_arbitration();
}

}  // namespace rover_msgs

#endif  // ROVER_MSGS__MSG__DETAIL__DRIVETRAIN_ARBITRATION__BUILDER_HPP_
