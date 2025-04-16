#ifndef __DDB__NODE__HPP__
#define __DDB__NODE__HPP__

#include "rovus_lib/camera_info.hpp"
#include "rovus_lib/macros.h"

#include "rclcpp/rclcpp.hpp"
#include <rover_msgs/srv/ddb_control.hpp>

class DDBControlNode : public rclcpp::Node
{

  enum class eSwitchState : size_t
  {
    OFF = 0,
    ON = 1,
  };

  struct switchInfo 
  {
    eSwitchState state;
    std::string mode;
    uint8_t dutyCycle;
    uint8_t frequency;
  };

  public:
    DDBControlNode();
    ~DDBControlNode() = default;

  private:

  void ddbControl(const rover_msgs::srv::DDBControl::Request& request_,
                  rover_msgs::srv::DDBControl::Response& response_);

  void toggleSwitch(const rover_msgs::srv::DDBControl::Request& request_);
  void togglePWM(const rover_msgs::srv::DDBControl::Request& request_);
  void modifyPWM(uint8_t duty_cycle, uint8_t frequency);

  rclcpp::Service<rover_msgs::srv::DDBControl>::SharedPtr _srv_control;

  eSwitchState _currentSwitchState;
  eSwitchState _currentPWMMode;
     
};

#endif