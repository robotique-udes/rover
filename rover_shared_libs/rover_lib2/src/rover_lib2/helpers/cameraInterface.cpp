#include "cameraInterface.hpp"
#include "rover_lib2/helpers/constants.hpp"
#include "rover_lib2/helpers/constants.hpp"
#include "rover_lib2/helpers/folders.hpp"
#include "rover_lib2/helpers/macros.hpp"
#include <utility>

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

void CameraInterface::setPTZCmd(rover_msgs::msg::CameraControl goalMsg_, size_t id_)
{
    if (id_ >= std::to_underlying(Constants::CameraInfo::eCamNames::eLast))
    {
        return;
    }

    if (!_isCamConcerned[id_])
    {
        _isCamConcerned[id_] = true;
    }

    _lastPtzCmdMsg[id_] = goalMsg_;
}

rover_msgs::msg::CameraControl CameraInterface::getPTZCmd(size_t id_) const
{
    return _lastPtzCmdMsg[id_];
}

void CameraInterface::setPTZConfig(rover_msgs::msg::CameraConfig configMsg_, size_t id_)
{
    if (id_ >= std::to_underlying(Constants::CameraInfo::eCamNames::eLast))
    {
        return;
    }

    if (!_isCamConcerned[id_])
    {
        _isCamConcerned[id_] = true;
    }

    _lastPtzConfigMsg[id_] = configMsg_;
}

rover_msgs::msg::CameraConfig CameraInterface::getPtzConfig(size_t id_) const
{
    return _lastPtzConfigMsg[id_];
}

void CameraInterface::setPowerCmd(rover_msgs::msg::CameraControl powerMsg_, size_t id_)
{
    if (id_ >= std::to_underlying(Constants::CameraInfo::eCamNames::eLast))
    {
        return;
    }

    if (!_isCamConcerned[id_])
    {
        _isCamConcerned[id_] = true;
    }

    _lastPowerMsg[id_] = powerMsg_;
}

rover_msgs::msg::CameraControl CameraInterface::getPowerCmd(size_t id_) const
{
    return _lastPowerMsg[id_];
}

void CameraInterface::release(size_t id_)
{
    _isCamConcerned[id_] = false;
}

bool CameraInterface::isGoalReached(size_t id_)
{
    return _isGoalReached[id_];
}

bool CameraInterface::isCamUnderControl(size_t id_)
{
    if (id_ >= std::to_underlying(Constants::CameraInfo::eCamNames::eLast))
    {
        return false;
    }

    if (_topicWithPriority[id_] == _ptzCommandTopic)
    {
        return true;
    }
    return false;
}

rover_msgs::msg::CameraControl CameraInterface::getLastPowerStatusMsg(size_t id_) const
{
    return _lastPowerStatusMsg[id_];
}

rover_msgs::msg::CameraControl CameraInterface::getLastPtzStatusMsg(size_t id_) const
{
    return _lastPtzStatusMsg[id_];
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
    for (size_t i = 0; i < _isCamConcerned.size(); i++)
    {
        if (_node && _isCamConcerned[i])
        {
            _pub_PTZCmd->publish(_lastPtzCmdMsg[i]);
        }
    }
}

void CameraInterface::CB_publishPtzConfig(void)
{
    for (size_t i = 0; i < _isCamConcerned.size(); i++)
    {
        if (_node && _isCamConcerned[i])
        {
            _pub_configCmd->publish(_lastPtzConfigMsg[i]);
        }
    }
}

void CameraInterface::CB_publishPowerCmd(void)
{
    for (size_t i = 0; i < _isCamConcerned.size(); i++)
    {
        if (_node && _isCamConcerned[i])
        {
            _pub_powerCmd->publish(_lastPowerMsg[i]);
        }
    }
}

void CameraInterface::CB_subscriberPowerStatus(rover_msgs::msg::CameraControl statusMsg_)
{
    size_t id = statusMsg_.id_cam;
    _lastPowerStatusMsg[id] = statusMsg_;
}

void CameraInterface::CB_subscriberPtzStatus(rover_msgs::msg::CameraControl statusMsg_)
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

void CameraInterface::CB_subscriberTopicWithPriority(rover_msgs::msg::TopicWithPriority topicLists_)
{
    for (size_t i = 0; i < _topicWithPriority.size(); i++)
    {
        _topicWithPriority[i] = topicLists_.topics[i];
    }
}
