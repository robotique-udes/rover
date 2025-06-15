#include "manager_node.hpp"
#include <optional>
#include <rclcpp/logging.hpp>
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
        size_t index = 0;

        for (auto& subscriber : _sub_PTZcmd)
        {
            subscriber = this->create_subscription<rover_msgs::msg::CameraControl>(
                Arbitration::PTZ_CMD_TOPIC[index],
                QOS_DEFAULT,
                [this, index](const rover_msgs::msg::CameraControl& PTZcmd_)
                {
                    this->_arbitration.CB_PTZcmdFiltering(PTZcmd_, index);
                    this->simu();
                });
            ++index;
        }
    }

    void ManagerNode::simu()
    {
        for (size_t i = 0; i < NUMBER_CAM; i++)
        {
            std::optional<rover_msgs::msg::CameraControl> cmd = _arbitration.getValidPTZcmdMsg(i);
            if (cmd.has_value())
            {
                size_t id = cmd.value().id_cam;
                float yaw = cmd.value().yaw;
                RCLCPP_WARN(this->get_logger(), "ID: %ld, YAW: %f", id,yaw);

            }
        }
    }
}  // namespace CameraManager
