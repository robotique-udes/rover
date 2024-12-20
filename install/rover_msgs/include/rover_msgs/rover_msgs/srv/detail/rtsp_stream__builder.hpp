// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rover_msgs:srv/RtspStream.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__SRV__DETAIL__RTSP_STREAM__BUILDER_HPP_
#define ROVER_MSGS__SRV__DETAIL__RTSP_STREAM__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rover_msgs/srv/detail/rtsp_stream__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rover_msgs
{

namespace srv
{

namespace builder
{

class Init_RtspStream_Request_demand
{
public:
  explicit Init_RtspStream_Request_demand(::rover_msgs::srv::RtspStream_Request & msg)
  : msg_(msg)
  {}
  ::rover_msgs::srv::RtspStream_Request demand(::rover_msgs::srv::RtspStream_Request::_demand_type arg)
  {
    msg_.demand = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_msgs::srv::RtspStream_Request msg_;
};

class Init_RtspStream_Request_stream_id
{
public:
  Init_RtspStream_Request_stream_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RtspStream_Request_demand stream_id(::rover_msgs::srv::RtspStream_Request::_stream_id_type arg)
  {
    msg_.stream_id = std::move(arg);
    return Init_RtspStream_Request_demand(msg_);
  }

private:
  ::rover_msgs::srv::RtspStream_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_msgs::srv::RtspStream_Request>()
{
  return rover_msgs::srv::builder::Init_RtspStream_Request_stream_id();
}

}  // namespace rover_msgs


namespace rover_msgs
{

namespace srv
{

namespace builder
{

class Init_RtspStream_Response_success
{
public:
  Init_RtspStream_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rover_msgs::srv::RtspStream_Response success(::rover_msgs::srv::RtspStream_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_msgs::srv::RtspStream_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_msgs::srv::RtspStream_Response>()
{
  return rover_msgs::srv::builder::Init_RtspStream_Response_success();
}

}  // namespace rover_msgs

#endif  // ROVER_MSGS__SRV__DETAIL__RTSP_STREAM__BUILDER_HPP_
