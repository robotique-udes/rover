#ifndef CAMERA_INTERFACE_HPP
#define CAMERA_INTERFACE_HPP

#include <rclcpp/subscription.hpp>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/camera_control.hpp"


class CameraInterface
{
    static constexpr uint8_t SEND_COMMAND_FREQUENCY = 5U;
    static constexpr float REICEVE_STATUS_FREQUENCY = 0.5F;
    #warning unused

  public:
    CameraInterface(std::shared_ptr<rclcpp::Node> node_, const std::string& commandTopic_, const std::string& statusTopic_);
    
    void setGoalMsg(rover_msgs::msg::CameraControl goalMsg_);
    rover_msgs::msg::CameraControl getGoalMsg(void) const;
    
    rover_msgs::msg::CameraControl getLastStatusMsg(void) const;

  private:
    void CB_publishCommand(const rover_msgs::msg::CameraControl& commandMsg_);
    void CB_subscriberStatus(rover_msgs::msg::CameraControl statusMsg_);

    rclcpp::Publisher<rover_msgs::msg::CameraControl>::SharedPtr _pub_command;
    rclcpp::Subscription<rover_msgs::msg::CameraControl>::SharedPtr _sub_status;
    
    rclcpp::TimerBase::SharedPtr _timer_pubCommand;

    rover_msgs::msg::CameraControl _goalMsg;
    rover_msgs::msg::CameraControl _lastStatusMsg;

    std::shared_ptr<rclcpp::Node> _node;
};

#endif //CAMERA_INTERFACE_HPP