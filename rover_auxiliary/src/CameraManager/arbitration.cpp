#include "arbitration.hpp"
#include <optional>
#include <rover_lib2/helpers/constants.hpp>
#include <rover_msgs/msg/detail/camera_control__builder.hpp>

#warning active topic publisher

namespace CameraManager
{

    Arbitration::Arbitration()
    {
        _isPTZTopicActive.fill(false);
        _highestPriorityLevelPtzCmd.fill(NUMBER_TOPIC_CAMERA_ARBITRATION - 1);
        _missingHighPriorityMsg.fill(0);
    }

    std::optional<rover_msgs::msg::CameraControl> Arbitration::getValidPTZcmdMsg(size_t camID_)
    {
        if (!_isPTZTopicActive.at(camID_))
        {
            return std::nullopt;
        }
        else
        {
            return _lastValidPTZCmd.at(camID_);
        }
    }

    std::optional<rover_msgs::msg::CameraConfig> Arbitration::getValidPTZConfig(size_t camID_)
    {
        if (!_isPTZTopicActive.at(camID_))
        {
            return std::nullopt;
        }
        else
        {
            return _lastValidPTZConfig.at(camID_);
        }
    }

    void Arbitration::CB_PTZCmdFiltering(rover_msgs::msg::CameraControl PTZcmd_, size_t priority)
    {
        size_t id = PTZcmd_.id_cam;

        if (_isPTZTopicActive.at(id) == false)
        {
            _isPTZTopicActive.at(id) = true;
            _highestPriorityLevelPtzCmd.at(id) = priority;
            topicWithPriority.at(id) = PTZ_CMD_TOPIC[priority];
            RCLCPP_INFO(rclcpp::get_logger("CAMERA_ARBITRATION"), "Cam %ld is now managed by %s", id, PTZ_CMD_TOPIC[priority]);
            _activeTopicCount++;
        }

        else if (priority < _highestPriorityLevelPtzCmd.at(id))
        {
            _highestPriorityLevelPtzCmd.at(id) = priority;
            topicWithPriority.at(id) = PTZ_CMD_TOPIC[priority];

            RCLCPP_DEBUG(rclcpp::get_logger("CAMERA_ARBITRATION"),
                         "Priority has shifted up to %s on cam %ld",
                         PTZ_CMD_TOPIC[priority],
                         id);
        }

        if (priority > _highestPriorityLevelPtzCmd.at(id) || !_isPTZTopicActive.at(id))
        {
            _missingHighPriorityMsg.at(id)++;

            size_t highPriorityMissingMsgThreshold = _activeTopicCount + HIGH_PRIORITY_MISSING_MSG_SAFETY_FACTOR;

            if (_missingHighPriorityMsg.at(id) > highPriorityMissingMsgThreshold)
            {
                _highestPriorityLevelPtzCmd.at(id) = priority;
                topicWithPriority.at(id) = PTZ_CMD_TOPIC[priority];

                RCLCPP_DEBUG(rclcpp::get_logger("CAMERA_ARBITRATION"),
                             "Priority has shifted down to %s on cam %ld",
                             PTZ_CMD_TOPIC[priority],
                             id);
            }
            return;
        }

        _missingHighPriorityMsg.at(id) = 0;
        _lastValidPTZCmd.at(id) = PTZcmd_;
    }

    void Arbitration::CB_PTZConfigFiltering(rover_msgs::msg::CameraConfig PTZConfig_)
    {
        size_t id = PTZConfig_.id_cam;
        _highestPriorityLevelPtzConfig.at(id) = _highestPriorityLevelPtzCmd.at(id);
    }
}  // namespace CameraManager