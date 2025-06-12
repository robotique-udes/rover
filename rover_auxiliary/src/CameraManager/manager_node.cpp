#include "manager_node.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraManager::ManagerNode>());
    rclcpp::shutdown();
    return 0;
}

namespace CameraManager
{

    ManagerNode::ManagerNode():Node("camera_manager")
    {

    }
} //namespace CameraManager
