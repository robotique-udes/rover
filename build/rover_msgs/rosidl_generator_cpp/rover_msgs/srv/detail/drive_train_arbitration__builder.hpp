// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rover_msgs:srv/DriveTrainArbitration.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__SRV__DETAIL__DRIVE_TRAIN_ARBITRATION__BUILDER_HPP_
#define ROVER_MSGS__SRV__DETAIL__DRIVE_TRAIN_ARBITRATION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rover_msgs/srv/detail/drive_train_arbitration__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rover_msgs
{

namespace srv
{

namespace builder
{

class Init_DriveTrainArbitration_Request_target_arbitration
{
public:
  Init_DriveTrainArbitration_Request_target_arbitration()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rover_msgs::srv::DriveTrainArbitration_Request target_arbitration(::rover_msgs::srv::DriveTrainArbitration_Request::_target_arbitration_type arg)
  {
    msg_.target_arbitration = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_msgs::srv::DriveTrainArbitration_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_msgs::srv::DriveTrainArbitration_Request>()
{
  return rover_msgs::srv::builder::Init_DriveTrainArbitration_Request_target_arbitration();
}

}  // namespace rover_msgs


namespace rover_msgs
{

namespace srv
{

namespace builder
{

class Init_DriveTrainArbitration_Response_current_arbitration
{
public:
  Init_DriveTrainArbitration_Response_current_arbitration()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rover_msgs::srv::DriveTrainArbitration_Response current_arbitration(::rover_msgs::srv::DriveTrainArbitration_Response::_current_arbitration_type arg)
  {
    msg_.current_arbitration = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_msgs::srv::DriveTrainArbitration_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_msgs::srv::DriveTrainArbitration_Response>()
{
  return rover_msgs::srv::builder::Init_DriveTrainArbitration_Response_current_arbitration();
}

}  // namespace rover_msgs

#endif  // ROVER_MSGS__SRV__DETAIL__DRIVE_TRAIN_ARBITRATION__BUILDER_HPP_
