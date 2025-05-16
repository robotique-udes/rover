// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rover_msgs:srv/PanoControl.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__SRV__DETAIL__PANO_CONTROL__BUILDER_HPP_
#define ROVER_MSGS__SRV__DETAIL__PANO_CONTROL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rover_msgs/srv/detail/pano_control__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rover_msgs
{

namespace srv
{

namespace builder
{

class Init_PanoControl_Request_ip_address
{
public:
  explicit Init_PanoControl_Request_ip_address(::rover_msgs::srv::PanoControl_Request & msg)
  : msg_(msg)
  {}
  ::rover_msgs::srv::PanoControl_Request ip_address(::rover_msgs::srv::PanoControl_Request::_ip_address_type arg)
  {
    msg_.ip_address = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_msgs::srv::PanoControl_Request msg_;
};

class Init_PanoControl_Request_photo
{
public:
  explicit Init_PanoControl_Request_photo(::rover_msgs::srv::PanoControl_Request & msg)
  : msg_(msg)
  {}
  Init_PanoControl_Request_ip_address photo(::rover_msgs::srv::PanoControl_Request::_photo_type arg)
  {
    msg_.photo = std::move(arg);
    return Init_PanoControl_Request_ip_address(msg_);
  }

private:
  ::rover_msgs::srv::PanoControl_Request msg_;
};

class Init_PanoControl_Request_stop
{
public:
  explicit Init_PanoControl_Request_stop(::rover_msgs::srv::PanoControl_Request & msg)
  : msg_(msg)
  {}
  Init_PanoControl_Request_photo stop(::rover_msgs::srv::PanoControl_Request::_stop_type arg)
  {
    msg_.stop = std::move(arg);
    return Init_PanoControl_Request_photo(msg_);
  }

private:
  ::rover_msgs::srv::PanoControl_Request msg_;
};

class Init_PanoControl_Request_start
{
public:
  Init_PanoControl_Request_start()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PanoControl_Request_stop start(::rover_msgs::srv::PanoControl_Request::_start_type arg)
  {
    msg_.start = std::move(arg);
    return Init_PanoControl_Request_stop(msg_);
  }

private:
  ::rover_msgs::srv::PanoControl_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_msgs::srv::PanoControl_Request>()
{
  return rover_msgs::srv::builder::Init_PanoControl_Request_start();
}

}  // namespace rover_msgs


namespace rover_msgs
{

namespace srv
{

namespace builder
{

class Init_PanoControl_Response_status_message
{
public:
  explicit Init_PanoControl_Response_status_message(::rover_msgs::srv::PanoControl_Response & msg)
  : msg_(msg)
  {}
  ::rover_msgs::srv::PanoControl_Response status_message(::rover_msgs::srv::PanoControl_Response::_status_message_type arg)
  {
    msg_.status_message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_msgs::srv::PanoControl_Response msg_;
};

class Init_PanoControl_Response_success
{
public:
  Init_PanoControl_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PanoControl_Response_status_message success(::rover_msgs::srv::PanoControl_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_PanoControl_Response_status_message(msg_);
  }

private:
  ::rover_msgs::srv::PanoControl_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_msgs::srv::PanoControl_Response>()
{
  return rover_msgs::srv::builder::Init_PanoControl_Response_success();
}

}  // namespace rover_msgs

#endif  // ROVER_MSGS__SRV__DETAIL__PANO_CONTROL__BUILDER_HPP_
