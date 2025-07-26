#include "manager_node.hpp"
#include <cstddef>
#include <optional>
#include <rclcpp/logging.hpp>
#include <rover_lib2/helpers/constants.hpp>
#include <rover_msgs/msg/detail/camera_control__struct.hpp>

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
            subscriber = this->create_subscription<rover_msgs::msg::CameraControl>(
                Arbitration::PTZ_CONFIG_TOPIC[index],
                QOS_DEFAULT,
                [this](const rover_msgs::msg::CameraControl& PTZConfig_)
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
                [this](const rover_msgs::msg::CameraControl& powerCmd_)
                {
                    CB_publishFilteredPowerCmd(powerCmd_);
                });
            ++index;
        }

        _publisher_filteredPTZCmd
            = this->create_publisher<rover_msgs::msg::CameraControl>(TOPIC_PTZ_COMMAND_MANAGER, QOS_DEFAULT);

        _timer_filtredPTZCmdPub
            = this->create_wall_timer(std::chrono::milliseconds(static_cast<size_t>(1000 / SEND_PTZ_COMMAND_FREQUENCY)),
                                      [this](void)
                                      {
                                          CB_publishFilteredPtzCmd();
                                      });

        _publisher_filteredPTZConfig
            = this->create_publisher<rover_msgs::msg::CameraControl>(TOPIC_PTZ_CONFIG_MANAGER, QOS_DEFAULT);

        _timer_filtredPTZConfigPub
            = this->create_wall_timer(std::chrono::milliseconds(static_cast<size_t>(1000 / SEND_PTZ_COMMAND_FREQUENCY)),
                                      [this](void)
                                      {
                                          CB_publishFilteredZPtzConfig();
                                      });

        _publisher_filteredPowerCmd
            = this->create_publisher<rover_msgs::msg::CameraControl>(TOPIC_POWER_COMMAND_MANAGER, QOS_DEFAULT);
    }

    void ManagerNode::CB_publishFilteredPtzCmd()
    {
        for (size_t i = 0; i < NUMBER_CAM; i++)
        {
            std::optional<rover_msgs::msg::CameraControl> cmd = _arbitration.getValidPTZcmdMsg(i);
            if (cmd.has_value())
            {
                size_t id = cmd.value().id_cam;
                float yaw = cmd.value().yaw;
                RCLCPP_INFO(this->get_logger(), "ID: %ld, YAW: %f", id, yaw);
                rover_msgs::msg::CameraControl msg;
                msg.id_cam = cmd.value().id_cam;
                msg.yaw = cmd.value().yaw;

                _publisher_filteredPTZCmd->publish(msg);
            }
        }
    }

    void ManagerNode::CB_publishFilteredZPtzConfig()
    {
        for (size_t i = 0; i < NUMBER_CAM; i++)
        {
            std::optional<rover_msgs::msg::CameraControl> cmd = _arbitration.getValidPTZConfig(i);
            if (cmd.has_value())
            {
                size_t id = cmd.value().id_cam;
                float yaw = cmd.value().yaw;
                RCLCPP_INFO(this->get_logger(), "ID: %ld, YAW: %f", id, yaw);
                rover_msgs::msg::CameraControl msg;
                msg.id_cam = cmd.value().id_cam;
                msg.yaw = cmd.value().yaw;

                _publisher_filteredPTZConfig->publish(msg);
            }
        }
    }

    void ManagerNode::CB_publishFilteredPowerCmd(rover_msgs::msg::CameraControl msg_)
    {
        _publisher_filteredPowerCmd->publish(msg_);
    }
}  // namespace CameraManager
