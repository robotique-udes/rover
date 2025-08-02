#include "manager_node.hpp"

#include <optional>
#include <rover_lib2/helpers/constants.hpp>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraManager::ManagerNode>());
    rclcpp::shutdown();
    return 0;
}

namespace CameraManager
{
    ManagerNode::ManagerNode():
        Node("camera_manager")
    {
        this->initSubs();
        this->initPubs();
    }

    void ManagerNode::CB_publishFilteredPtzCmd()
    {
        for (size_t i = 0; i < std::to_underlying(Constants::CameraInfo::eCamNames::eLast); i++)
        {
            std::optional<rover_msgs::msg::CameraControl> cmd = _arbitration.getValidPTZcmdMsg(i);
            if (cmd.has_value())
            {
                _publisher_filteredPTZCmd->publish(cmd.value());
            }
        }
    }

    void ManagerNode::CB_publishFilteredZPtzConfig()
    {
        for (size_t i = 0; i < std::to_underlying(Constants::CameraInfo::eCamNames::eLast); i++)
        {
            std::optional<rover_msgs::msg::CameraConfig> config = _arbitration.getValidPTZConfig(i);
            if (config.has_value())
            {
                _publisher_filteredPTZConfig->publish(config.value());
            }
        }
    }

    void ManagerNode::CB_storePowerCmd(rover_msgs::msg::CameraControl msg_, size_t index_)
    {
        if (index_ >= std::to_underlying(Constants::CameraInfo::eCamNames::eLast))
        {
            return;
        }
        _lastPowerMsg[index_] = msg_;

        _publisher_filteredPowerCmd->publish(msg_);
    }

    void ManagerNode::CB_publishTopicWithPriority()
    {
        rover_msgs::msg::TopicWithPriority msg;

        for (size_t i = 0; i < _arbitration.topicWithPriority.size(); i++)
        {
            msg.topics.push_back(_arbitration.topicWithPriority[i]);
        }
        _publisher_topicWithPriority->publish(msg);
    }

    void ManagerNode::CB_publishFilteredPowerCmd()
    {
        bool power_on = false;
        for (size_t i = 0; i < _lastPowerMsg.size(); i++)
        {
            if (_lastPowerMsg[i].power_on)
            {
                power_on = true;
            }
        }
        rover_msgs::msg::CameraControl nextMsg;
        nextMsg.power_on = power_on;
        _publisher_filteredPowerCmd->publish(nextMsg);
    }

    void ManagerNode::initSubs()
    {
        size_t index = 0;

        for (auto& subscriber : _sub_PTZCmd)
        {
            subscriber = this->create_subscription<rover_msgs::msg::CameraControl>(
                Arbitration::PTZ_CMD_TOPIC[index],
                QOS_DEFAULT,
                [this, index](const rover_msgs::msg::CameraControl& PTZcmd_)
                {
                    this->_arbitration.CB_PTZCmdFiltering(PTZcmd_, index);
                });
            ++index;
        }

        index = 0;

        for (auto& subscriber : _sub_PTZConfig)
        {
            subscriber = this->create_subscription<rover_msgs::msg::CameraConfig>(
                Arbitration::PTZ_CONFIG_TOPIC[index],
                QOS_DEFAULT,
                [this](const rover_msgs::msg::CameraConfig& PTZConfig_)
                {
                    this->_arbitration.CB_PTZConfigFiltering(PTZConfig_);
                });
            ++index;
        }

        index = 0;
        for (auto& subscriber : _sub_powerCmd)
        {
            subscriber = this->create_subscription<rover_msgs::msg::CameraControl>(
                Arbitration::POWER_CMD_TOPIC[index],
                QOS_DEFAULT,
                [this, index](const rover_msgs::msg::CameraControl& powerCmd_)
                {
                    CB_storePowerCmd(powerCmd_, index);
                });
            ++index;
        }
    }

    void ManagerNode::initPubs()
    {
        _publisher_filteredPTZCmd
            = this->create_publisher<rover_msgs::msg::CameraControl>(TOPIC_PTZ_COMMAND_MANAGER, QOS_DEFAULT);

        _timer_filtredPTZCmdPub = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<size_t>(1000 / Constants::CameraInfo::SEND_COMMAND_PTZ_FREQUENCY)),
            [this](void)
            {
                CB_publishFilteredPtzCmd();
            });

        _publisher_filteredPTZConfig
            = this->create_publisher<rover_msgs::msg::CameraConfig>(TOPIC_PTZ_CONFIG_MANAGER, QOS_DEFAULT);

        _timer_filtredPTZConfigPub = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<size_t>(1000.F / Constants::CameraInfo::SEND_CONFIG_PTZ_FREQUENCY)),
            [this](void)
            {
                CB_publishFilteredZPtzConfig();
            });

        _publisher_filteredPowerCmd
            = this->create_publisher<rover_msgs::msg::CameraControl>(TOPIC_POWER_COMMAND_MANAGER, QOS_DEFAULT);

        _timer_filtredPowerCmdPub = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<size_t>(1000.F / Constants::CameraInfo::SEND_COMMAND_POWER_FREQUENCY)),
            [this](void)
            {
                CB_publishFilteredPowerCmd();
            });

        _publisher_topicWithPriority
            = this->create_publisher<rover_msgs::msg::TopicWithPriority>(TOPIC_WITH_PRIORITY, QOS_DEFAULT);

        _timer_topicWithPriorityPub = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<size_t>(1000.F / Constants::CameraInfo::SEND_COMMAND_PTZ_FREQUENCY)),
            [this](void)
            {
                CB_publishTopicWithPriority();
            });
    }
}  // namespace CameraManager
