// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rover_msgs:srv/ScreenshotControl.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__SRV__DETAIL__SCREENSHOT_CONTROL__BUILDER_HPP_
#define ROVER_MSGS__SRV__DETAIL__SCREENSHOT_CONTROL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rover_msgs/srv/detail/screenshot_control__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rover_msgs
{

namespace srv
{

namespace builder
{

class Init_ScreenshotControl_Request_metadata
{
public:
  explicit Init_ScreenshotControl_Request_metadata(::rover_msgs::srv::ScreenshotControl_Request & msg)
  : msg_(msg)
  {}
  ::rover_msgs::srv::ScreenshotControl_Request metadata(::rover_msgs::srv::ScreenshotControl_Request::_metadata_type arg)
  {
    msg_.metadata = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_msgs::srv::ScreenshotControl_Request msg_;
};

class Init_ScreenshotControl_Request_ip_address
{
public:
  explicit Init_ScreenshotControl_Request_ip_address(::rover_msgs::srv::ScreenshotControl_Request & msg)
  : msg_(msg)
  {}
  Init_ScreenshotControl_Request_metadata ip_address(::rover_msgs::srv::ScreenshotControl_Request::_ip_address_type arg)
  {
    msg_.ip_address = std::move(arg);
    return Init_ScreenshotControl_Request_metadata(msg_);
  }

private:
  ::rover_msgs::srv::ScreenshotControl_Request msg_;
};

class Init_ScreenshotControl_Request_name
{
public:
  explicit Init_ScreenshotControl_Request_name(::rover_msgs::srv::ScreenshotControl_Request & msg)
  : msg_(msg)
  {}
  Init_ScreenshotControl_Request_ip_address name(::rover_msgs::srv::ScreenshotControl_Request::_name_type arg)
  {
    msg_.name = std::move(arg);
    return Init_ScreenshotControl_Request_ip_address(msg_);
  }

private:
  ::rover_msgs::srv::ScreenshotControl_Request msg_;
};

class Init_ScreenshotControl_Request_start
{
public:
  Init_ScreenshotControl_Request_start()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ScreenshotControl_Request_name start(::rover_msgs::srv::ScreenshotControl_Request::_start_type arg)
  {
    msg_.start = std::move(arg);
    return Init_ScreenshotControl_Request_name(msg_);
  }

private:
  ::rover_msgs::srv::ScreenshotControl_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_msgs::srv::ScreenshotControl_Request>()
{
  return rover_msgs::srv::builder::Init_ScreenshotControl_Request_start();
}

}  // namespace rover_msgs


namespace rover_msgs
{

namespace srv
{

namespace builder
{

class Init_ScreenshotControl_Response_status_message
{
public:
  explicit Init_ScreenshotControl_Response_status_message(::rover_msgs::srv::ScreenshotControl_Response & msg)
  : msg_(msg)
  {}
  ::rover_msgs::srv::ScreenshotControl_Response status_message(::rover_msgs::srv::ScreenshotControl_Response::_status_message_type arg)
  {
    msg_.status_message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_msgs::srv::ScreenshotControl_Response msg_;
};

class Init_ScreenshotControl_Response_success
{
public:
  Init_ScreenshotControl_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ScreenshotControl_Response_status_message success(::rover_msgs::srv::ScreenshotControl_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_ScreenshotControl_Response_status_message(msg_);
  }

private:
  ::rover_msgs::srv::ScreenshotControl_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_msgs::srv::ScreenshotControl_Response>()
{
  return rover_msgs::srv::builder::Init_ScreenshotControl_Response_success();
}

}  // namespace rover_msgs

#endif  // ROVER_MSGS__SRV__DETAIL__SCREENSHOT_CONTROL__BUILDER_HPP_
