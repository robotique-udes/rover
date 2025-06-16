#include "cameraInterface.hpp"
#include <cstddef>
#include <rover_lib2/helpers/constants.hpp>

CameraInterface::CameraInterface(std::shared_ptr<rclcpp::Node> node_,
                                 const std::string& commandTopic_,
                                 const std::string& statusTopic_)
{
    _node = node_;

    _isCamConcerned.fill(false);

    if (_node)
    {
        _pub_command = _node->create_publisher<rover_msgs::msg::CameraControl>(commandTopic_, QOS_DEFAULT);
        _sub_status = _node->create_subscription<rover_msgs::msg::CameraControl>(statusTopic_,
                                                                                 QOS_DEFAULT,
                                                                                 [this](rover_msgs::msg::CameraControl msg_)
                                                                                 {
                                                                                     this->CB_subscriberStatus(msg_);
                                                                                 });

        _timer_pubCommand = _node->create_wall_timer(std::chrono::milliseconds(static_cast<size_t>(1000 / SEND_COMMAND_FREQUENCY)),
                                                     [this](void)
                                                     {
                                                         this->CB_publishCommand();
                                                     });
    }
}

void CameraInterface::setGoalMsg(rover_msgs::msg::CameraControl goalMsg_, size_t id_)
{
    if (!_isCamConcerned.at(id_))
    {
        _isCamConcerned.at(id_) = true;
    }

    _goalMsg.at(id_) = goalMsg_;
}

void CameraInterface::forgetGoal(size_t id_)
{
    _isCamConcerned.at(id_) = false;
}

rover_msgs::msg::CameraControl CameraInterface::getGoalMsg(size_t id_) const
{
    return _goalMsg.at(id_);
}

rover_msgs::msg::CameraControl CameraInterface::getLastStatusMsg(size_t id_) const
{
    return _lastStatusMsg.at(id_);
}

void CameraInterface::CB_publishCommand(void)
{
    for (size_t i = 0; i < NUMBER_CAM; i++)
    {
        if (_node && _isCamConcerned.at(i))
        {
            _pub_command->publish(_goalMsg.at(i));
        }
    }
}

void CameraInterface::CB_subscriberStatus(rover_msgs::msg::CameraControl statusMsg_)
{
    size_t id = statusMsg_.id_cam;
    _lastStatusMsg.at(id) = statusMsg_;
}