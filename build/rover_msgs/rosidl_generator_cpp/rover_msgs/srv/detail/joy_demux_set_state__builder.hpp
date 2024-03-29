// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rover_msgs:srv/JoyDemuxSetState.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__SRV__DETAIL__JOY_DEMUX_SET_STATE__BUILDER_HPP_
#define ROVER_MSGS__SRV__DETAIL__JOY_DEMUX_SET_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rover_msgs/srv/detail/joy_demux_set_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rover_msgs
{

namespace srv
{

namespace builder
{

class Init_JoyDemuxSetState_Request_force
{
public:
  explicit Init_JoyDemuxSetState_Request_force(::rover_msgs::srv::JoyDemuxSetState_Request & msg)
  : msg_(msg)
  {}
  ::rover_msgs::srv::JoyDemuxSetState_Request force(::rover_msgs::srv::JoyDemuxSetState_Request::_force_type arg)
  {
    msg_.force = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_msgs::srv::JoyDemuxSetState_Request msg_;
};

class Init_JoyDemuxSetState_Request_destination
{
public:
  explicit Init_JoyDemuxSetState_Request_destination(::rover_msgs::srv::JoyDemuxSetState_Request & msg)
  : msg_(msg)
  {}
  Init_JoyDemuxSetState_Request_force destination(::rover_msgs::srv::JoyDemuxSetState_Request::_destination_type arg)
  {
    msg_.destination = std::move(arg);
    return Init_JoyDemuxSetState_Request_force(msg_);
  }

private:
  ::rover_msgs::srv::JoyDemuxSetState_Request msg_;
};

class Init_JoyDemuxSetState_Request_controller_type
{
public:
  Init_JoyDemuxSetState_Request_controller_type()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_JoyDemuxSetState_Request_destination controller_type(::rover_msgs::srv::JoyDemuxSetState_Request::_controller_type_type arg)
  {
    msg_.controller_type = std::move(arg);
    return Init_JoyDemuxSetState_Request_destination(msg_);
  }

private:
  ::rover_msgs::srv::JoyDemuxSetState_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_msgs::srv::JoyDemuxSetState_Request>()
{
  return rover_msgs::srv::builder::Init_JoyDemuxSetState_Request_controller_type();
}

}  // namespace rover_msgs


namespace rover_msgs
{

namespace srv
{

namespace builder
{

class Init_JoyDemuxSetState_Response_success
{
public:
  Init_JoyDemuxSetState_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rover_msgs::srv::JoyDemuxSetState_Response success(::rover_msgs::srv::JoyDemuxSetState_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_msgs::srv::JoyDemuxSetState_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_msgs::srv::JoyDemuxSetState_Response>()
{
  return rover_msgs::srv::builder::Init_JoyDemuxSetState_Response_success();
}

}  // namespace rover_msgs

#endif  // ROVER_MSGS__SRV__DETAIL__JOY_DEMUX_SET_STATE__BUILDER_HPP_
