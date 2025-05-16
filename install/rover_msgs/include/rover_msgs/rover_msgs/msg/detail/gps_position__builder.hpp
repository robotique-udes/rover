// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rover_msgs:msg/GpsPosition.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__MSG__DETAIL__GPS_POSITION__BUILDER_HPP_
#define ROVER_MSGS__MSG__DETAIL__GPS_POSITION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rover_msgs/msg/detail/gps_position__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rover_msgs
{

namespace msg
{

namespace builder
{

class Init_GpsPosition_longitude
{
public:
  explicit Init_GpsPosition_longitude(::rover_msgs::msg::GpsPosition & msg)
  : msg_(msg)
  {}
  ::rover_msgs::msg::GpsPosition longitude(::rover_msgs::msg::GpsPosition::_longitude_type arg)
  {
    msg_.longitude = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_msgs::msg::GpsPosition msg_;
};

class Init_GpsPosition_latitude
{
public:
  Init_GpsPosition_latitude()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GpsPosition_longitude latitude(::rover_msgs::msg::GpsPosition::_latitude_type arg)
  {
    msg_.latitude = std::move(arg);
    return Init_GpsPosition_longitude(msg_);
  }

private:
  ::rover_msgs::msg::GpsPosition msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_msgs::msg::GpsPosition>()
{
  return rover_msgs::msg::builder::Init_GpsPosition_latitude();
}

}  // namespace rover_msgs

#endif  // ROVER_MSGS__MSG__DETAIL__GPS_POSITION__BUILDER_HPP_
