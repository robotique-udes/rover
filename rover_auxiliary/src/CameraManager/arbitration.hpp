#ifndef ARBITRATION_HPP
#define ARBITRATION_HPP

#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/camera_control.hpp"
#include "rover_msgs/msg/camera_config.hpp"
#include "rover_lib2/helpers/constants.hpp"

namespace CameraManager
{
    class Arbitration
    {
      private:
        static constexpr const size_t NUMBER_TOPIC_CAMERA_ARBITRATION = Constants::CameraInfo::NUMBER_TOPIC_CAMERA_ARBITRATION;
        static_assert(NUMBER_TOPIC_CAMERA_ARBITRATION > 1, "Number of topic managing PTZ on cameras lower than 1");
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

        std::optional<rover_msgs::msg::CameraControl> getValidPTZcmdMsg(size_t camID_) const;
        std::optional<rover_msgs::msg::CameraConfig> getValidPTZConfig(size_t camID_) const;

        void CB_PTZCmdFiltering(rover_msgs::msg::CameraControl PTZCmd_, size_t priority_);
        void CB_PTZConfigFiltering(rover_msgs::msg::CameraConfig PTZConfig_, size_t priority_);

        std::array<std::string, std::to_underlying(Constants::CameraInfo::eCamNames::eLast)> topicWithPriority;

      private:
        std::array<rover_msgs::msg::CameraControl, std::to_underlying(Constants::CameraInfo::eCamNames::eLast)> _lastValidPTZCmd;
        std::array<rover_msgs::msg::CameraConfig, std::to_underlying(Constants::CameraInfo::eCamNames::eLast)>
            _lastValidPTZConfig;

        std::array<size_t, std::to_underlying(Constants::CameraInfo::eCamNames::eLast)> _currentPriorityLevelPtzCmd;

        std::array<bool, std::to_underlying(Constants::CameraInfo::eCamNames::eLast)> _isPTZTopicActive = {false};

        std::array<size_t, std::to_underlying(Constants::CameraInfo::eCamNames::eLast)> _missedHighPriorityMsg;

        size_t _activeTopicCount = 0U;
    };

}  // namespace CameraManager

#endif  // ARBITRATION_HPP