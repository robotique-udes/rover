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
        static constexpr const double GOAL_MARGIN = 0.01;


        static constexpr float SEND_PTZ_COMMAND_FREQUENCY = 5.F;
        static constexpr float SEND_PTZ_CONFIG_FREQUENCY = .5F;
        static constexpr float SEND_POWER_COMMAND_FREQUENCY = .5F;

        static constexpr const char* TOPIC_PTZ_COMMAND_MANAGER = "rover/camera/PTZ_cmd/manager";
        static constexpr const char* TOPIC_PTZ_CONFIG_MANAGER = "rover/camera/PTZ_config/manager";
        static constexpr const char* TOPIC_POWER_COMMAND_MANAGER = "rover/camera/power_cmd/manager";

        static constexpr const char* TOPIC_PTZ_STATUS = "rover/camera/PTZ_status";


      public:
        ManagerNode();
        void CB_publishFilteredPtzCmd(void);
        void CB_publishFilteredZPtzConfig(void);
        void CB_publishFilteredPowerCmd(rover_msgs::msg::CameraControl msg_);

      private:
        Arbitration _arbitration;

        std::array<rclcpp::Subscription<rover_msgs::msg::CameraControl>::SharedPtr, NUMBER_TOPIC> _sub_PTZCmd;
        std::array<rclcpp::Subscription<rover_msgs::msg::CameraControl>::SharedPtr, NUMBER_TOPIC> _sub_PTZConfig;
        std::array<rclcpp::Subscription<rover_msgs::msg::CameraControl>::SharedPtr, NUMBER_TOPIC> _sub_powerCmd;

        rclcpp::Publisher<rover_msgs::msg::CameraControl>::SharedPtr _publisher_filteredPTZCmd;
        rclcpp::Publisher<rover_msgs::msg::CameraControl>::SharedPtr _publisher_filteredPTZConfig;
        rclcpp::Publisher<rover_msgs::msg::CameraControl>::SharedPtr _publisher_filteredPowerCmd;

        rclcpp::TimerBase::SharedPtr _timer_filtredPTZCmdPub;
        rclcpp::TimerBase::SharedPtr _timer_filtredPTZConfigPub;
        rclcpp::TimerBase::SharedPtr _timer_filtredPowerCmdPub;

    };
}  // namespace CameraManager
#endif  // MANAGER_NODE_HPP