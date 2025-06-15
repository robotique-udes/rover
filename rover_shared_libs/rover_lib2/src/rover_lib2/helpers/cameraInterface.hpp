#ifndef CAMERA_INTERFACE_HPP
#define CAMERA_INTERFACE_HPP

#include <cstddef>
#include <cstdint>
#include <rclcpp/subscription.hpp>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/camera_control.hpp"


class CameraInterface
{
    static constexpr uint8_t SEND_COMMAND_FREQUENCY = 5U;
    static constexpr float REICEVE_STATUS_FREQUENCY = 0.5F;

    static constexpr uint8_t NUMBER_CAM = 5;

    #warning unused

    #warning IF STATUS IS NOT RECEIVED A MESSAGE IS CREATED WITH ALL 0 (default) WE WOULD WANT TO MESSAGE IN THAT CASE...

    #warning WE WOULD WHAT TO BE ABLE TO HAVE A FCT TO STOP THE PUBLISHMENT...

  public:
    CameraInterface(std::shared_ptr<rclcpp::Node> node_, const std::string& commandTopic_, const std::string& statusTopic_);
    
    void setGoalMsg(rover_msgs::msg::CameraControl goalMsg_,size_t id_);
    rover_msgs::msg::CameraControl getGoalMsg(size_t id_) const;

    void forgetGoal(size_t id_);
    
    rover_msgs::msg::CameraControl getLastStatusMsg(size_t id_) const;

  private:
    void CB_publishCommand(void);
    void CB_subscriberStatus(rover_msgs::msg::CameraControl statusMsg_);

    rclcpp::Publisher<rover_msgs::msg::CameraControl>::SharedPtr _pub_command;
    rclcpp::Subscription<rover_msgs::msg::CameraControl>::SharedPtr _sub_status;
    
    rclcpp::TimerBase::SharedPtr _timer_pubCommand;

    std::array<rover_msgs::msg::CameraControl,NUMBER_CAM> _goalMsg;
    std::array<bool, NUMBER_CAM> _isCamConcerned;
    std::array<rover_msgs::msg::CameraControl,NUMBER_CAM> _lastStatusMsg;

    std::shared_ptr<rclcpp::Node> _node;
};

#endif //CAMERA_INTERFACE_HPP