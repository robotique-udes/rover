#ifndef MANAGER_NODE_HPP
#define MANAGER_NODE_HPP

#include "goal_manager.hpp"
#include "arbitration.hpp"
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/node.hpp>
#include "rover_msgs/msg/camera_control.hpp"

namespace CameraManager
{
    class ManagerNode : public rclcpp::Node
    {
        static constexpr const size_t NUMBER_TOPIC = 2;
        static constexpr const size_t NUMBER_CAM = 5;

      public:
        ManagerNode();
        void simu(void);

      private:
        Arbitration _arbitration;

        std::array<rclcpp::Subscription<rover_msgs::msg::CameraControl>::SharedPtr, NUMBER_TOPIC> _sub_PTZcmd;
    };

}  // namespace CameraManager
#endif  // MANAGER_NODE_HPP