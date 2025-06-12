#include "cameraInterface.hpp"
#include <rover_lib2/helpers/constants.hpp>

CameraInterface::CameraInterface(std::shared_ptr<rclcpp::Node> node_, const std::string& commandTopic_, const std::string& statusTopic_)
{
    _node = node_;

    if(_node){
        _pub_command = _node->create_publisher<rover_msgs::msg::CameraControl>(commandTopic_, QOS_DEFAULT);
        _sub_status = _node->create_subscription<rover_msgs::msg::CameraControl>(statusTopic_,
                                                                                QOS_DEFAULT,
                                                                                [this](rover_msgs::msg::CameraControl msg_)
                                                                                {
                                                                                    this->CB_subscriberStatus(msg_);
                                                                                });
        _timer_pubCommand = _node->create_wall_timer(std::chrono::milliseconds(1000/SEND_COMMAND_FREQUENCY), [this](void)
        {
            this->CB_publishCommand(_goalMsg);
        });
    }
}

void CameraInterface::setGoalMsg(rover_msgs::msg::CameraControl goalMsg_)
{
    _goalMsg = goalMsg_;
}

rover_msgs::msg::CameraControl CameraInterface::getGoalMsg(void) const
{
    return _goalMsg;
}

rover_msgs::msg::CameraControl CameraInterface::getLastStatusMsg(void) const
{
    return _lastStatusMsg;
}

void CameraInterface::CB_publishCommand(const rover_msgs::msg::CameraControl& commandMsg_)
{
    if(_node)
    {
        _pub_command->publish(commandMsg_);
    }
}

void CameraInterface::CB_subscriberStatus(rover_msgs::msg::CameraControl statusMsg_)
{
    _lastStatusMsg = statusMsg_;
}