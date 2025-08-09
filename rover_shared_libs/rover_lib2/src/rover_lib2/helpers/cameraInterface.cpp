#include "cameraInterface.hpp"
#include "rover_lib2/helpers/constants.hpp"
#include "rover_lib2/helpers/constants.hpp"
#include "rover_lib2/helpers/folders.hpp"
#include "rover_lib2/helpers/macros.hpp"

#include <utility>

#if defined(ROS)
CameraInterface::CameraInterface(std::shared_ptr<rclcpp::Node> node_,
                                 const std::string& ptzCommandTopic_,
                                 const std::string& ptzConfigTopic_,
                                 const std::string& powerCommandTopic_)
{
    _node = node_;
    _ptzCommandTopic = ptzCommandTopic_;
    _ptzConfigTopic = ptzConfigTopic_;
    _powerCommandTopic = powerCommandTopic_;

    if (_node)
    {
        this->initTimers();
        this->initPub();
        this->initSub();
    }
}

void CameraInterface::setPTZCmd(const rover_msgs::msg::CameraControl& goalMsg_, Constants::CameraInfo::eCamNames id_)
{
    if (id_ == Constants::CameraInfo::eCamNames::eLast)
    {
        return;
    }

    if (!_isCamConcerned[std::to_underlying(id_)])
    {
        _isCamConcerned[std::to_underlying(id_)] = true;
    }

    _lastPtzCmdMsg[std::to_underlying(id_)] = goalMsg_;
}

rover_msgs::msg::CameraControl CameraInterface::getPTZCmd(Constants::CameraInfo::eCamNames id_) const
{
    return _lastPtzCmdMsg[std::to_underlying(id_)];
}

void CameraInterface::setPTZConfig(const rover_msgs::msg::CameraConfig& configMsg_, Constants::CameraInfo::eCamNames id_)
{
    if (id_ == Constants::CameraInfo::eCamNames::eLast)
    {
        return;
    }

    if (!_isCamConcerned[std::to_underlying(id_)])
    {
        _isCamConcerned[std::to_underlying(id_)] = true;
    }

    _lastPtzConfigMsg[std::to_underlying(id_)] = configMsg_;
}

rover_msgs::msg::CameraConfig CameraInterface::getPtzConfig(Constants::CameraInfo::eCamNames id_) const
{
    return _lastPtzConfigMsg[std::to_underlying(id_)];
}

void CameraInterface::setPowerCmd(const rover_msgs::msg::CameraControl& powerMsg_, Constants::CameraInfo::eCamNames id_)
{
    if (id_ == Constants::CameraInfo::eCamNames::eLast)
    {
        return;
    }

    if (!_isCamConcerned[std::to_underlying(id_)])
    {
        _isCamConcerned[std::to_underlying(id_)] = true;
    }

    _lastPowerMsg[std::to_underlying(id_)] = powerMsg_;
}

rover_msgs::msg::CameraControl CameraInterface::getPowerCmd(Constants::CameraInfo::eCamNames id_) const
{
    return _lastPowerMsg[std::to_underlying(id_)];
}

void CameraInterface::release(Constants::CameraInfo::eCamNames id_)
{
    _isCamConcerned[std::to_underlying(id_)] = false;
}

bool CameraInterface::isGoalReached(Constants::CameraInfo::eCamNames id_)
{
    return _isGoalReached[std::to_underlying(id_)];
}

bool CameraInterface::isCamUnderControl(Constants::CameraInfo::eCamNames id_)
{
    if (id_ == Constants::CameraInfo::eCamNames::eLast)
    {
        return false;
    }

    if (_topicWithPriority[std::to_underlying(id_)] == _ptzCommandTopic)
    {
        return true;
    }
    return false;
}

rover_msgs::msg::CameraControl CameraInterface::getLastPowerStatusMsg(Constants::CameraInfo::eCamNames id_) const
{
    return _lastPowerStatusMsg[std::to_underlying(id_)];
}

rover_msgs::msg::CameraControl CameraInterface::getLastPtzStatusMsg(Constants::CameraInfo::eCamNames id_) const
{
    return _lastPtzStatusMsg[std::to_underlying(id_)];
}

void CameraInterface::initTimers()
{
    _timer_pubPTZCmd
        = _node->create_wall_timer(std::chrono::milliseconds(static_cast<size_t>(1000.F / SEND_COMMAND_PTZ_FREQUENCY)),
                                   [this](void)
                                   {
                                       this->CB_publishPtzCmd();
                                   });

    _timer_pubPowerCmd
        = _node->create_wall_timer(std::chrono::milliseconds(static_cast<size_t>(1000.F / SEND_COMMAND_POWER_FREQUENCY)),
                                   [this](void)
                                   {
                                       this->CB_publishPowerCmd();
                                   });

    _timer_pubPTZConfig
        = _node->create_wall_timer(std::chrono::milliseconds(static_cast<size_t>(1000.F / SEND_CONFIG_PTZ_FREQUENCY)),
                                   [this](void)
                                   {
                                       this->CB_publishPtzConfig();
                                   });
}

void CameraInterface::initSub()
{
    _sub_powerStatus = _node->create_subscription<rover_msgs::msg::CameraControl>(POWER_STATUS_TOPIC,
                                                                                  QOS_DEFAULT,
                                                                                  [this](rover_msgs::msg::CameraControl msg_)
                                                                                  {
                                                                                      this->CB_subscriberPowerStatus(msg_);
                                                                                  });

    _sub_PTZStatus = _node->create_subscription<rover_msgs::msg::CameraControl>(PTZ_STATUS_TOPIC,
                                                                                QOS_DEFAULT,
                                                                                [this](rover_msgs::msg::CameraControl msg_)
                                                                                {
                                                                                    this->CB_subscriberPtzStatus(msg_);
                                                                                });

    _sub_topicWithPriority
        = _node->create_subscription<rover_msgs::msg::TopicWithPriority>(TOPIC_WITH_PRIORITY,
                                                                         QOS_DEFAULT,
                                                                         [this](rover_msgs::msg::TopicWithPriority msg_)
                                                                         {
                                                                             this->CB_subscriberTopicWithPriority(msg_);
                                                                         });
}

void CameraInterface::initPub()
{
    _pub_PTZCmd = _node->create_publisher<rover_msgs::msg::CameraControl>(_ptzCommandTopic, QOS_DEFAULT);
    _pub_configCmd = _node->create_publisher<rover_msgs::msg::CameraConfig>(_ptzConfigTopic, QOS_DEFAULT);
    _pub_powerCmd = _node->create_publisher<rover_msgs::msg::CameraControl>(_powerCommandTopic, QOS_DEFAULT);
}

void CameraInterface::CB_publishPtzCmd(void)
{
    if (!_node)
    {
        return;
    }

    for (size_t i = 0; i < _isCamConcerned.size(); ++i)
    {
        if (_isCamConcerned[i])
        {
            _pub_PTZCmd->publish(_lastPtzCmdMsg[i]);
        }
    }
}

void CameraInterface::CB_publishPtzConfig(void)
{
    if (!_node)
    {
        return;
    }

    for (size_t i = 0; i < _isCamConcerned.size(); ++i)
    {
        if (_isCamConcerned[i])
        {
            _pub_configCmd->publish(_lastPtzConfigMsg[i]);
        }
    }
}

void CameraInterface::CB_publishPowerCmd(void)
{
    if (!_node)
    {
        return;
    }

    for (size_t i = 0; i < _isCamConcerned.size(); ++i)
    {
        if (_isCamConcerned[i])
        {
            _pub_powerCmd->publish(_lastPowerMsg[i]);
        }
    }
}

void CameraInterface::CB_subscriberPowerStatus(const rover_msgs::msg::CameraControl& statusMsg_)
{
    size_t id = statusMsg_.id_cam;
    _lastPowerStatusMsg[id] = statusMsg_;
}

void CameraInterface::CB_subscriberPtzStatus(const rover_msgs::msg::CameraControl& statusMsg_)
{
    size_t id = statusMsg_.id_cam;
    _lastPowerStatusMsg[id] = statusMsg_;
    float currentYaw = statusMsg_.yaw;

    if (IN_ERROR(currentYaw, EPSILON, _lastPtzCmdMsg[id].yaw))
    {
        _isGoalReached[id] = true;
    }
    else
    {
        _isGoalReached[id] = false;
    }
}

void CameraInterface::CB_subscriberTopicWithPriority(const rover_msgs::msg::TopicWithPriority& topicLists_)
{
    for (size_t i = 0; i < _topicWithPriority.size(); ++i)
    {
        _topicWithPriority[i] = topicLists_.topics[i];
    }
}

#endif  // defined(ROS)
