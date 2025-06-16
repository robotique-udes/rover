#ifndef MANAGER_NODE_HPP
#define MANAGER_NODE_HPP

#include "goal_manager.hpp"
#include "arbitration.hpp"
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>
#include <rover_msgs/msg/detail/camera_control__struct.hpp>
#include "rover_msgs/msg/camera_control.hpp"

namespace CameraManager
{
    class ManagerNode : public rclcpp::Node
    {
        static constexpr const size_t NUMBER_TOPIC = 2;
        static constexpr const size_t NUMBER_CAM = 5;

        static constexpr float SEND_COMMAND_FREQUENCY = 1.F;

        static constexpr const char* TOPIC_SEND_PTZCOMMAND_MANAGER = "rover/camera/PTZcmd/manager";

      public:
        ManagerNode();
        void CB_publishFilteredPtzCmd(void);

      private:
        Arbitration _arbitration;

        std::array<rclcpp::Subscription<rover_msgs::msg::CameraControl>::SharedPtr, NUMBER_TOPIC> _sub_PTZcmd;
        rclcpp::Publisher<rover_msgs::msg::CameraControl>::SharedPtr _publisher_filteredPTZcmd;
        rclcpp::TimerBase::SharedPtr _timer_filtredPTZcmdPub;
    };

}  // namespace CameraManager
#endif  // MANAGER_NODE_HPP