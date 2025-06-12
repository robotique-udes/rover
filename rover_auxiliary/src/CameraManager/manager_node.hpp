#ifndef MANAGER_NODE_HPP
#define MANAGER_NODE_HPP

#include "goal_manager.hpp"
#include "arbitration.hpp"
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/node.hpp>
//#include "rover_msgs/msg/camera_control.hpp"

namespace CameraManager 
{
    class ManagerNode: public rclcpp::Node
    {
        public:
            ManagerNode();
    };

} //namespace CameraManager
#endif //MANAGER_NODE_HPP