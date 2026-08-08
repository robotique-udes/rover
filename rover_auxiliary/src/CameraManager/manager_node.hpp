#ifndef MANAGER_NODE_HPP
#define MANAGER_NODE_HPP

#include "arbitration.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rover_lib2/helpers/constants.hpp"
#include "rover_msgs/msg/camera_control.hpp"
#include "rover_msgs/msg/camera_config.hpp"
#include "rover_msgs/msg/topic_with_priority.hpp"
#include "rover_msgs/srv/camera_ir.hpp"
#include <utility>

namespace CameraManager
{
    class ManagerNode : public rclcpp::Node
    {
        static constexpr const char* TOPIC_PTZ_COMMAND_MANAGER = "/rover/camera/PTZ_cmd/manager";
        static constexpr const char* TOPIC_PTZ_CONFIG_MANAGER = "/rover/camera/PTZ_config/manager";
        static constexpr const char* TOPIC_POWER_COMMAND_MANAGER = "/rover/camera/power_cmd/manager";

        static constexpr const char* TOPIC_PTZ_STATUS = "/rover/camera/PTZ_status";
        static constexpr const char* TOPIC_POWER_STATUS = "/rover/camera/power_status";

        static constexpr const char* TOPIC_WITH_PRIORITY = "/rover/camera/topic_with_priority";

        static constexpr const char* SERVICE_IR = "rover/camera/IR";

      public:
        ManagerNode();

      private:
        Arbitration _arbitration;

        void CB_publishFilteredPtzCmd(void);
        void CB_publishFilteredZPtzConfig(void);
        void CB_storePowerCmd(rover_msgs::msg::CameraControl msg_, size_t topicIndex_);
        void CB_publishFilteredPowerCmd(void);
        void CB_publishTopicWithPriority(void);

        void CB_srvIR(const rover_msgs::srv::CameraIR::Request::SharedPtr request_,
                      rover_msgs::srv::CameraIR::Response::SharedPtr response_);

        void initSubs(void);
        void initPubs(void);

        std::array<rclcpp::Subscription<rover_msgs::msg::CameraControl>::SharedPtr,
                   Constants::CameraInfo::NUMBER_TOPIC_CAMERA_ARBITRATION>
            _sub_PTZCmd;
        std::array<rclcpp::Subscription<rover_msgs::msg::CameraConfig>::SharedPtr,
                   Constants::CameraInfo::NUMBER_TOPIC_CAMERA_ARBITRATION>
            _sub_PTZConfig;
        std::array<rclcpp::Subscription<rover_msgs::msg::CameraControl>::SharedPtr,
                   Constants::CameraInfo::NUMBER_TOPIC_CAMERA_ARBITRATION>
            _sub_powerCmd;

        std::array<std::array<rover_msgs::msg::CameraControl, Constants::CameraInfo::NUMBER_TOPIC_CAMERA_ARBITRATION>,
                   std::to_underlying(Constants::CameraInfo::eCamNames::eLast)>
            _lastPowerMsg;

        rclcpp::Publisher<rover_msgs::msg::CameraControl>::SharedPtr _publisher_filteredPTZCmd;
        rclcpp::Publisher<rover_msgs::msg::CameraConfig>::SharedPtr _publisher_filteredPTZConfig;
        rclcpp::Publisher<rover_msgs::msg::CameraControl>::SharedPtr _publisher_filteredPowerCmd;

        rclcpp::Publisher<rover_msgs::msg::TopicWithPriority>::SharedPtr _publisher_topicWithPriority;

        rclcpp::Service<rover_msgs::srv::CameraIR>::SharedPtr _srv_IR;

        rclcpp::TimerBase::SharedPtr _timer_filtredPTZCmdPub;
        rclcpp::TimerBase::SharedPtr _timer_filtredPTZConfigPub;
        rclcpp::TimerBase::SharedPtr _timer_filtredPowerCmdPub;
        rclcpp::TimerBase::SharedPtr _timer_topicWithPriorityPub;
    };
}  // namespace CameraManager
#endif  // MANAGER_NODE_HPP