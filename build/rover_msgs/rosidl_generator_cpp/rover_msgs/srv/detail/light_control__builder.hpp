// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rover_msgs:srv/LightControl.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__SRV__DETAIL__LIGHT_CONTROL__BUILDER_HPP_
#define ROVER_MSGS__SRV__DETAIL__LIGHT_CONTROL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rover_msgs/srv/detail/light_control__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rover_msgs
{

namespace srv
{

namespace builder
{

class Init_LightControl_Request_enable
{
public:
  explicit Init_LightControl_Request_enable(::rover_msgs::srv::LightControl_Request & msg)
  : msg_(msg)
  {}
  ::rover_msgs::srv::LightControl_Request enable(::rover_msgs::srv::LightControl_Request::_enable_type arg)
  {
    msg_.enable = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_msgs::srv::LightControl_Request msg_;
};

class Init_LightControl_Request_index
{
public:
  Init_LightControl_Request_index()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LightControl_Request_enable index(::rover_msgs::srv::LightControl_Request::_index_type arg)
  {
    msg_.index = std::move(arg);
    return Init_LightControl_Request_enable(msg_);
  }

private:
  ::rover_msgs::srv::LightControl_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_msgs::srv::LightControl_Request>()
{
  return rover_msgs::srv::builder::Init_LightControl_Request_index();
}

}  // namespace rover_msgs


namespace rover_msgs
{

namespace srv
{

namespace builder
{

class Init_LightControl_Response_success
{
public:
  Init_LightControl_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rover_msgs::srv::LightControl_Response success(::rover_msgs::srv::LightControl_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_msgs::srv::LightControl_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_msgs::srv::LightControl_Response>()
{
  return rover_msgs::srv::builder::Init_LightControl_Response_success();
}

}  // namespace rover_msgs

#endif  // ROVER_MSGS__SRV__DETAIL__LIGHT_CONTROL__BUILDER_HPP_
