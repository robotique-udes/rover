#ifndef __DDB__NODE__HPP__
#define __DDB__NODE__HPP__

#include "rovus_lib/camera_info.hpp"
#include "rovus_lib/macros.h"

#include "rclcpp/rclcpp.hpp"
#include <rover_msgs/srv/ddb_control.hpp>

class DDBControlNode : public rclcpp::Node
{
  public:
    DDBControlNode();
    ~DDBControlNode() = default;

  private:

  void ddbControl(const rover_msgs::srv::DDBControl::Request& request_,
                  rover_msgs::srv::DDBControl::Response& response_);

  rclcpp::Service<rover_msgs::srv::DDBControl>::SharedPtr _srv_control;
     
};

#endif