#ifndef ARBITRATION_HPP
#define ARBITRATION_HPP

#include "rclcpp/rclcpp.hpp"
#include <cstddef>
#include <cstdint>
#include "rover_msgs/msg/camera_control.hpp"

namespace CameraManager
{

    class Arbitration
    {
        static constexpr const size_t NUMBER_CAM = 5;
        static constexpr const size_t NUMBER_TOPIC = 2;
        static constexpr const size_t LOWEST_PRIORITY_LEVEL = NUMBER_TOPIC-1;
        static constexpr const size_t HIGH_PRIORITY_MISSING_MSG_SAFETY_FACTOR = 2;

      public:
        static constexpr const char* PTZ_CMD_TOPIC[NUMBER_TOPIC] = {"rover/camera/PTZcmd/panorama", "rover/camera/PTZcmd/GUI"};

        Arbitration();

        std::optional<rover_msgs::msg::CameraControl> getValidPTZcmdMsg(size_t camID_);

        void CB_PTZcmdFiltering(rover_msgs::msg::CameraControl PTZcmd_, size_t priority);

      private:
        std::array<rover_msgs::msg::CameraControl, NUMBER_CAM> _lastValidPTZcmd;

        std::array<size_t, NUMBER_CAM> _highestPriorityLevel;

        std::array<bool, NUMBER_CAM> _isPTZTopicActive;

        std::array<size_t, NUMBER_CAM> _missingHighPriorityMsg;

        size_t _activeTopicCount = 0;

    };

}  // namespace CameraManager

#endif  // ARBITRATION_HPP