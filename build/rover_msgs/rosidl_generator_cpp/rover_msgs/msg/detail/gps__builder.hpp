// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rover_msgs:msg/Gps.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__MSG__DETAIL__GPS__BUILDER_HPP_
#define ROVER_MSGS__MSG__DETAIL__GPS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rover_msgs/msg/detail/gps__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rover_msgs
{

namespace msg
{

namespace builder
{

class Init_Gps_heading
{
public:
  explicit Init_Gps_heading(::rover_msgs::msg::Gps & msg)
  : msg_(msg)
  {}
  ::rover_msgs::msg::Gps heading(::rover_msgs::msg::Gps::_heading_type arg)
  {
    msg_.heading = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_msgs::msg::Gps msg_;
};

class Init_Gps_satellite
{
public:
  explicit Init_Gps_satellite(::rover_msgs::msg::Gps & msg)
  : msg_(msg)
  {}
  Init_Gps_heading satellite(::rover_msgs::msg::Gps::_satellite_type arg)
  {
    msg_.satellite = std::move(arg);
    return Init_Gps_heading(msg_);
  }

private:
  ::rover_msgs::msg::Gps msg_;
};

class Init_Gps_speed
{
public:
  explicit Init_Gps_speed(::rover_msgs::msg::Gps & msg)
  : msg_(msg)
  {}
  Init_Gps_satellite speed(::rover_msgs::msg::Gps::_speed_type arg)
  {
    msg_.speed = std::move(arg);
    return Init_Gps_satellite(msg_);
  }

private:
  ::rover_msgs::msg::Gps msg_;
};

class Init_Gps_heading_track
{
public:
  explicit Init_Gps_heading_track(::rover_msgs::msg::Gps & msg)
  : msg_(msg)
  {}
  Init_Gps_speed heading_track(::rover_msgs::msg::Gps::_heading_track_type arg)
  {
    msg_.heading_track = std::move(arg);
    return Init_Gps_speed(msg_);
  }

private:
  ::rover_msgs::msg::Gps msg_;
};

class Init_Gps_heading_gps
{
public:
  explicit Init_Gps_heading_gps(::rover_msgs::msg::Gps & msg)
  : msg_(msg)
  {}
  Init_Gps_heading_track heading_gps(::rover_msgs::msg::Gps::_heading_gps_type arg)
  {
    msg_.heading_gps = std::move(arg);
    return Init_Gps_heading_track(msg_);
  }

private:
  ::rover_msgs::msg::Gps msg_;
};

class Init_Gps_height
{
public:
  explicit Init_Gps_height(::rover_msgs::msg::Gps & msg)
  : msg_(msg)
  {}
  Init_Gps_heading_gps height(::rover_msgs::msg::Gps::_height_type arg)
  {
    msg_.height = std::move(arg);
    return Init_Gps_heading_gps(msg_);
  }

private:
  ::rover_msgs::msg::Gps msg_;
};

class Init_Gps_longitude
{
public:
  explicit Init_Gps_longitude(::rover_msgs::msg::Gps & msg)
  : msg_(msg)
  {}
  Init_Gps_height longitude(::rover_msgs::msg::Gps::_longitude_type arg)
  {
    msg_.longitude = std::move(arg);
    return Init_Gps_height(msg_);
  }

private:
  ::rover_msgs::msg::Gps msg_;
};

class Init_Gps_latitude
{
public:
  Init_Gps_latitude()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Gps_longitude latitude(::rover_msgs::msg::Gps::_latitude_type arg)
  {
    msg_.latitude = std::move(arg);
    return Init_Gps_longitude(msg_);
  }

private:
  ::rover_msgs::msg::Gps msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_msgs::msg::Gps>()
{
  return rover_msgs::msg::builder::Init_Gps_latitude();
}

}  // namespace rover_msgs

#endif  // ROVER_MSGS__MSG__DETAIL__GPS__BUILDER_HPP_
