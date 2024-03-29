// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rover_msgs:msg/JoyDemuxStatus.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__MSG__DETAIL__JOY_DEMUX_STATUS__BUILDER_HPP_
#define ROVER_MSGS__MSG__DETAIL__JOY_DEMUX_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rover_msgs/msg/detail/joy_demux_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rover_msgs
{

namespace msg
{

namespace builder
{

class Init_JoyDemuxStatus_controller_secondary_topic
{
public:
  explicit Init_JoyDemuxStatus_controller_secondary_topic(::rover_msgs::msg::JoyDemuxStatus & msg)
  : msg_(msg)
  {}
  ::rover_msgs::msg::JoyDemuxStatus controller_secondary_topic(::rover_msgs::msg::JoyDemuxStatus::_controller_secondary_topic_type arg)
  {
    msg_.controller_secondary_topic = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_msgs::msg::JoyDemuxStatus msg_;
};

class Init_JoyDemuxStatus_controller_main_topic
{
public:
  Init_JoyDemuxStatus_controller_main_topic()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_JoyDemuxStatus_controller_secondary_topic controller_main_topic(::rover_msgs::msg::JoyDemuxStatus::_controller_main_topic_type arg)
  {
    msg_.controller_main_topic = std::move(arg);
    return Init_JoyDemuxStatus_controller_secondary_topic(msg_);
  }

private:
  ::rover_msgs::msg::JoyDemuxStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_msgs::msg::JoyDemuxStatus>()
{
  return rover_msgs::msg::builder::Init_JoyDemuxStatus_controller_main_topic();
}

}  // namespace rover_msgs

#endif  // ROVER_MSGS__MSG__DETAIL__JOY_DEMUX_STATUS__BUILDER_HPP_
