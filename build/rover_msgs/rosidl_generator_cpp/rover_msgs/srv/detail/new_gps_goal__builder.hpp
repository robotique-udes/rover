// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rover_msgs:srv/NewGpsGoal.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__SRV__DETAIL__NEW_GPS_GOAL__BUILDER_HPP_
#define ROVER_MSGS__SRV__DETAIL__NEW_GPS_GOAL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rover_msgs/srv/detail/new_gps_goal__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rover_msgs
{

namespace srv
{

namespace builder
{

class Init_NewGpsGoal_Request_waypoints
{
public:
  explicit Init_NewGpsGoal_Request_waypoints(::rover_msgs::srv::NewGpsGoal_Request & msg)
  : msg_(msg)
  {}
  ::rover_msgs::srv::NewGpsGoal_Request waypoints(::rover_msgs::srv::NewGpsGoal_Request::_waypoints_type arg)
  {
    msg_.waypoints = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_msgs::srv::NewGpsGoal_Request msg_;
};

class Init_NewGpsGoal_Request_index
{
public:
  explicit Init_NewGpsGoal_Request_index(::rover_msgs::srv::NewGpsGoal_Request & msg)
  : msg_(msg)
  {}
  Init_NewGpsGoal_Request_waypoints index(::rover_msgs::srv::NewGpsGoal_Request::_index_type arg)
  {
    msg_.index = std::move(arg);
    return Init_NewGpsGoal_Request_waypoints(msg_);
  }

private:
  ::rover_msgs::srv::NewGpsGoal_Request msg_;
};

class Init_NewGpsGoal_Request_type
{
public:
  Init_NewGpsGoal_Request_type()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NewGpsGoal_Request_index type(::rover_msgs::srv::NewGpsGoal_Request::_type_type arg)
  {
    msg_.type = std::move(arg);
    return Init_NewGpsGoal_Request_index(msg_);
  }

private:
  ::rover_msgs::srv::NewGpsGoal_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_msgs::srv::NewGpsGoal_Request>()
{
  return rover_msgs::srv::builder::Init_NewGpsGoal_Request_type();
}

}  // namespace rover_msgs


namespace rover_msgs
{

namespace srv
{

namespace builder
{

class Init_NewGpsGoal_Response_route
{
public:
  explicit Init_NewGpsGoal_Response_route(::rover_msgs::srv::NewGpsGoal_Response & msg)
  : msg_(msg)
  {}
  ::rover_msgs::srv::NewGpsGoal_Response route(::rover_msgs::srv::NewGpsGoal_Response::_route_type arg)
  {
    msg_.route = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_msgs::srv::NewGpsGoal_Response msg_;
};

class Init_NewGpsGoal_Response_status
{
public:
  explicit Init_NewGpsGoal_Response_status(::rover_msgs::srv::NewGpsGoal_Response & msg)
  : msg_(msg)
  {}
  Init_NewGpsGoal_Response_route status(::rover_msgs::srv::NewGpsGoal_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_NewGpsGoal_Response_route(msg_);
  }

private:
  ::rover_msgs::srv::NewGpsGoal_Response msg_;
};

class Init_NewGpsGoal_Response_success
{
public:
  Init_NewGpsGoal_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NewGpsGoal_Response_status success(::rover_msgs::srv::NewGpsGoal_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_NewGpsGoal_Response_status(msg_);
  }

private:
  ::rover_msgs::srv::NewGpsGoal_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_msgs::srv::NewGpsGoal_Response>()
{
  return rover_msgs::srv::builder::Init_NewGpsGoal_Response_success();
}

}  // namespace rover_msgs

#endif  // ROVER_MSGS__SRV__DETAIL__NEW_GPS_GOAL__BUILDER_HPP_
