#ifndef ARBITRATION_HPP
#define ARBITRATION_HPP

#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/camera_control.hpp"
#include "rover_msgs/msg/camera_config.hpp"
#include <rover_msgs/msg/detail/camera_config__struct.hpp>

namespace CameraManager
{
    class Arbitration
    {
      public:
        static constexpr const size_t NUMBER_TOPIC_CAMERA_ARBITRATION = 2;

      private:
        static constexpr const size_t NUMBER_CAM = 5;
        static constexpr const size_t LOWEST_PRIORITY_LEVEL = NUMBER_TOPIC_CAMERA_ARBITRATION - 1;
        static constexpr const size_t HIGH_PRIORITY_MISSING_MSG_SAFETY_FACTOR = 2;

      public:
        static constexpr const char* PTZ_CMD_TOPIC[NUMBER_TOPIC_CAMERA_ARBITRATION]
            = {"/rover/camera/PTZ_cmd/panorama", "/rover/camera/PTZ_cmd/GUI"};
        static constexpr const char* PTZ_CONFIG_TOPIC[NUMBER_TOPIC_CAMERA_ARBITRATION]
            = {"/rover/camera/PTZ_config/panorama", "/rover/camera/PTZ_config/GUI"};
        static constexpr const char* POWER_CMD_TOPIC[NUMBER_TOPIC_CAMERA_ARBITRATION]
            = {"/rover/camera/power_cmd/panorama", "/rover/camera/power_cmd/GUI"};

        Arbitration();

        std::optional<rover_msgs::msg::CameraControl> getValidPTZcmdMsg(size_t camID_);
        std::optional<rover_msgs::msg::CameraConfig> getValidPTZConfig(size_t camID_);

        void CB_PTZCmdFiltering(rover_msgs::msg::CameraControl PTZCmd_, size_t priority);
        void CB_PTZConfigFiltering(rover_msgs::msg::CameraConfig PTZConfig_);

        std::array<std::string, NUMBER_CAM> topicWithPriority;

      private:
        std::array<rover_msgs::msg::CameraControl, NUMBER_CAM> _lastValidPTZCmd;
        std::array<rover_msgs::msg::CameraConfig, NUMBER_CAM> _lastValidPTZConfig;

        std::array<size_t, NUMBER_CAM> _highestPriorityLevelPtzCmd;
        std::array<size_t, NUMBER_CAM> _highestPriorityLevelPtzConfig;

        std::array<bool, NUMBER_CAM> _isPTZTopicActive;

        std::array<size_t, NUMBER_CAM> _missingHighPriorityMsg;

        size_t _activeTopicCount = 0;
    };

}  // namespace CameraManager

#endif  // ARBITRATION_HPP