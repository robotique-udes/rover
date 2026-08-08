#include "manager_node.hpp"

#include <optional>
#include <rover_lib2/helpers/constants.hpp>
#include <rover_lib2/cameras/IM50L35.hpp>

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
                _publisher_filteredPTZCmd->publish(*cmd);
            }
        }
    }

    void ManagerNode::CB_publishFilteredZPtzConfig()
    {
        for (size_t idCam = 0; idCam < std::to_underlying(Constants::CameraInfo::eCamNames::eLast); idCam++)
        {
            std::optional<rover_msgs::msg::CameraConfig> config = _arbitration.getValidPTZConfig(idCam);
            if (config.has_value())
            {
                _publisher_filteredPTZConfig->publish(*config);
            }
        }
    }

    void ManagerNode::CB_storePowerCmd(rover_msgs::msg::CameraControl msg_, size_t topicIndex_)
    {
        if (msg_.id_cam >= std::to_underlying(Constants::CameraInfo::eCamNames::eLast))
        {
            return;
        }
        _lastPowerMsg[msg_.id_cam][topicIndex_] = msg_;
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
        for (size_t idCam = 0; idCam < std::to_underlying(Constants::CameraInfo::eCamNames::eLast); ++idCam)
        {
            bool power_on = false;
            for (const rover_msgs::msg::CameraControl& msgOnEachTopic : _lastPowerMsg[idCam])
            {
                if (msgOnEachTopic.power_on)
                {
                    power_on = true;
                }
            }
            rover_msgs::msg::CameraControl nextMsg;
            nextMsg.id_cam = idCam;
            nextMsg.power_on = power_on;
            _publisher_filteredPowerCmd->publish(nextMsg);
        }
    }

    void ManagerNode::initSubs()
    {
        size_t topicIndex = 0;  // == priority

        for (auto& subscriber : _sub_PTZCmd)
        {
            subscriber = this->create_subscription<rover_msgs::msg::CameraControl>(
                Arbitration::PTZ_CMD_TOPIC[topicIndex],
                QOS_CAMERA,
                [this, topicIndex](const rover_msgs::msg::CameraControl& PTZcmd_)
                {
                    this->_arbitration.CB_PTZCmdFiltering(PTZcmd_, topicIndex);
                });
            ++topicIndex;
        }

        topicIndex = 0;

        for (auto& subscriber : _sub_PTZConfig)
        {
            subscriber = this->create_subscription<rover_msgs::msg::CameraConfig>(
                Arbitration::PTZ_CONFIG_TOPIC[topicIndex],
                QOS_CAMERA,
                [this, topicIndex](const rover_msgs::msg::CameraConfig& PTZConfig_)
                {
                    this->_arbitration.CB_PTZConfigFiltering(PTZConfig_, topicIndex);
                });
            ++topicIndex;
        }

        topicIndex = 0;
        for (auto& subscriber : _sub_powerCmd)
        {
            subscriber = this->create_subscription<rover_msgs::msg::CameraControl>(
                Arbitration::POWER_CMD_TOPIC[topicIndex],
                QOS_CAMERA,
                [this, topicIndex](const rover_msgs::msg::CameraControl& powerCmd_)
                {
                    CB_storePowerCmd(powerCmd_, topicIndex);
                });
            ++topicIndex;
        }
    }

    void ManagerNode::initPubs()
    {
        _publisher_filteredPTZCmd = this->create_publisher<rover_msgs::msg::CameraControl>(TOPIC_PTZ_COMMAND_MANAGER, QOS_CAMERA);

        _timer_filtredPTZCmdPub = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<size_t>(1000 / Constants::CameraInfo::SEND_COMMAND_PTZ_FREQUENCY)),
            [this](void)
            {
                this->CB_publishFilteredPtzCmd();
            });

        _publisher_filteredPTZConfig
            = this->create_publisher<rover_msgs::msg::CameraConfig>(TOPIC_PTZ_CONFIG_MANAGER, QOS_CAMERA);

        _timer_filtredPTZConfigPub = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<size_t>(1000.F / Constants::CameraInfo::SEND_CONFIG_PTZ_FREQUENCY)),
            [this](void)
            {
                this->CB_publishFilteredZPtzConfig();
            });

        _publisher_filteredPowerCmd
            = this->create_publisher<rover_msgs::msg::CameraControl>(TOPIC_POWER_COMMAND_MANAGER, QOS_CAMERA);

        _timer_filtredPowerCmdPub = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<size_t>(1000.F / Constants::CameraInfo::SEND_COMMAND_POWER_FREQUENCY)),
            [this](void)
            {
                this->CB_publishFilteredPowerCmd();
            });

        _publisher_topicWithPriority
            = this->create_publisher<rover_msgs::msg::TopicWithPriority>(TOPIC_WITH_PRIORITY, QOS_CAMERA);

        _timer_topicWithPriorityPub = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<size_t>(1000.F / Constants::CameraInfo::SEND_COMMAND_PTZ_FREQUENCY)),
            [this](void)
            {
                this->CB_publishTopicWithPriority();
            });
    }

    void ManagerNode::CB_srvIR(const rover_msgs::srv::CameraIR::Request& request_, rover_msgs::srv::CameraIR::Response& response_)
    {
        (void)response_;
        IM50L35::IRModes IRMode;
        switch (request_.ir_mode)
        {
            case rover_msgs::srv::CameraIR::Request::DAYMODE:
                IRMode = IM50L35::IRModes::DAY;
                break;
            case rover_msgs::srv::CameraIR::Request::NIGHTMODE:
                IRMode = IM50L35::IRModes::NIGHT;
                break;
            default:
                IRMode = IM50L35::IRModes::DAY;
                break;
        }

        IM50L35::setIR(request_.ip, IRMode, request_.ir_enable);
    }

}  // namespace CameraManager
