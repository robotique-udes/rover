#ifndef MANAGER_NODE_HPP
#define MANAGER_NODE_HPP

#include "arbitration.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/camera_control.hpp"
#include "rover_msgs/msg/camera_config.hpp"
#include "rover_msgs/msg/topic_with_priority.hpp"

namespace CameraManager
{
    class ManagerNode : public rclcpp::Node
    {
        static constexpr const size_t NUMBER_CAM = 5;

        static constexpr float SEND_PTZ_COMMAND_FREQUENCY = 5.F;
        static constexpr float SEND_PTZ_CONFIG_FREQUENCY = .5F;
        static constexpr float SEND_POWER_COMMAND_FREQUENCY = .5F;
        static constexpr float SEND_TOPIC_PRIORITY_FREQUENCY = 5.F;

        static constexpr const char* TOPIC_PTZ_COMMAND_MANAGER = "/rover/camera/PTZ_cmd/manager";
        static constexpr const char* TOPIC_PTZ_CONFIG_MANAGER = "/rover/camera/PTZ_config/manager";
        static constexpr const char* TOPIC_POWER_COMMAND_MANAGER = "/rover/camera/power_cmd/manager";

        static constexpr const char* TOPIC_PTZ_STATUS = "/rover/camera/PTZ_status";
        static constexpr const char* TOPIC_POWER_STATUS = "/rover/camera/power_status";

        static constexpr const char* TOPIC_WITH_PRIORITY = "/rover/camera/topic_with_priority";

      public:
        ManagerNode();

      private:
        Arbitration _arbitration;

        void CB_publishFilteredPtzCmd(void);
        void CB_publishFilteredZPtzConfig(void);
        void CB_publishFilteredPowerCmd(rover_msgs::msg::CameraControl msg_);
        void CB_publishTopicWithPriority(void);

        void initSub(void);
        void initPub(void);

        std::array<rclcpp::Subscription<rover_msgs::msg::CameraControl>::SharedPtr, Arbitration::NUMBER_TOPIC_CAMERA_ARBITRATION>
            _sub_PTZCmd;
        std::array<rclcpp::Subscription<rover_msgs::msg::CameraConfig>::SharedPtr, Arbitration::NUMBER_TOPIC_CAMERA_ARBITRATION>
            _sub_PTZConfig;
        std::array<rclcpp::Subscription<rover_msgs::msg::CameraControl>::SharedPtr, Arbitration::NUMBER_TOPIC_CAMERA_ARBITRATION>
            _sub_powerCmd;

        rclcpp::Publisher<rover_msgs::msg::CameraControl>::SharedPtr _publisher_filteredPTZCmd;
        rclcpp::Publisher<rover_msgs::msg::CameraConfig>::SharedPtr _publisher_filteredPTZConfig;
        rclcpp::Publisher<rover_msgs::msg::CameraControl>::SharedPtr _publisher_filteredPowerCmd;

        rclcpp::Publisher<rover_msgs::msg::TopicWithPriority>::SharedPtr _publisher_topicWithPriority;

        rclcpp::TimerBase::SharedPtr _timer_filtredPTZCmdPub;
        rclcpp::TimerBase::SharedPtr _timer_filtredPTZConfigPub;
        rclcpp::TimerBase::SharedPtr _timer_filtredPowerCmdPub;
        rclcpp::TimerBase::SharedPtr _timer_topicWithPriorityPub;
    };
}  // namespace CameraManager
#endif  // MANAGER_NODE_HPP