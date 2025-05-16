// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rover_msgs:srv/CompassCalibration.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__SRV__DETAIL__COMPASS_CALIBRATION__BUILDER_HPP_
#define ROVER_MSGS__SRV__DETAIL__COMPASS_CALIBRATION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rover_msgs/srv/detail/compass_calibration__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rover_msgs
{

namespace srv
{

namespace builder
{

class Init_CompassCalibration_Request_angle_offset
{
public:
  Init_CompassCalibration_Request_angle_offset()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rover_msgs::srv::CompassCalibration_Request angle_offset(::rover_msgs::srv::CompassCalibration_Request::_angle_offset_type arg)
  {
    msg_.angle_offset = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_msgs::srv::CompassCalibration_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_msgs::srv::CompassCalibration_Request>()
{
  return rover_msgs::srv::builder::Init_CompassCalibration_Request_angle_offset();
}

}  // namespace rover_msgs


namespace rover_msgs
{

namespace srv
{

namespace builder
{

class Init_CompassCalibration_Response_success
{
public:
  Init_CompassCalibration_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rover_msgs::srv::CompassCalibration_Response success(::rover_msgs::srv::CompassCalibration_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_msgs::srv::CompassCalibration_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_msgs::srv::CompassCalibration_Response>()
{
  return rover_msgs::srv::builder::Init_CompassCalibration_Response_success();
}

}  // namespace rover_msgs

#endif  // ROVER_MSGS__SRV__DETAIL__COMPASS_CALIBRATION__BUILDER_HPP_
