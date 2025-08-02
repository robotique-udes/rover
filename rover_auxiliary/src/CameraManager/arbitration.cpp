#include "arbitration.hpp"

#include <optional>
#include <rover_lib2/helpers/constants.hpp>
#include <utility>

namespace CameraManager
{

    Arbitration::Arbitration()
    {
        _currentPriorityLevelPtzCmd.fill(LOWEST_PRIORITY_LEVEL);
        _missedHighPriorityMsg.fill(0);
    }

    std::optional<rover_msgs::msg::CameraControl> Arbitration::getValidPTZcmdMsg(size_t camID_) const
    {
        if (camID_ >= std::to_underlying(Constants::CameraInfo::eCamNames::eLast))
        {
            return std::nullopt;
        }
        if (!_isPTZTopicActive[camID_])
        {
            return std::nullopt;
        }
        else
        {
            return _lastValidPTZCmd[camID_];
        }
    }

    std::optional<rover_msgs::msg::CameraConfig> Arbitration::getValidPTZConfig(size_t camID_) const
    {
        if (camID_ >= std::to_underlying(Constants::CameraInfo::eCamNames::eLast))
        {
            return std::nullopt;
        }
        if (!_isPTZTopicActive[camID_])
        {
            return std::nullopt;
        }
        else
        {
            return _lastValidPTZConfig[camID_];
        }
    }

    void Arbitration::CB_PTZCmdFiltering(rover_msgs::msg::CameraControl PTZcmd_, size_t priority_)
    {
        size_t id = PTZcmd_.id_cam;

        if (id >= std::to_underlying(Constants::CameraInfo::eCamNames::eLast))
        {
            return;
        }

        if (_isPTZTopicActive[id] == false)
        {
            _isPTZTopicActive[id] = true;
            _currentPriorityLevelPtzCmd[id] = priority_;
            topicWithPriority[id] = PTZ_CMD_TOPIC[priority_];
            RCLCPP_INFO(rclcpp::get_logger("CAMERA_ARBITRATION"), "Cam %ld is now managed by %s", id, PTZ_CMD_TOPIC[priority_]);
            _activeTopicCount++;
        }

        else if (priority_ < _currentPriorityLevelPtzCmd[id])
        {
            _currentPriorityLevelPtzCmd[id] = priority_;
            topicWithPriority[id] = PTZ_CMD_TOPIC[priority_];

            RCLCPP_INFO(rclcpp::get_logger("CAMERA_ARBITRATION"),
                        "Priority has shifted up to %s on cam %ld",
                        PTZ_CMD_TOPIC[priority_],
                        id);
        }

        if (priority_ > _currentPriorityLevelPtzCmd[id] || !_isPTZTopicActive[id])
        {
            _missedHighPriorityMsg[id]++;

            size_t highPriorityMissingMsgThreshold = _activeTopicCount + HIGH_PRIORITY_MISSING_MSG_SAFETY_FACTOR;

            if (_missedHighPriorityMsg[id] > highPriorityMissingMsgThreshold)
            {
                _currentPriorityLevelPtzCmd[id] = priority_;
                topicWithPriority[id] = PTZ_CMD_TOPIC[priority_];

                RCLCPP_DEBUG(rclcpp::get_logger("CAMERA_ARBITRATION"),
                             "Priority has shifted down to %s on cam %ld",
                             PTZ_CMD_TOPIC[priority_],
                             id);
            }
            return;
        }

        _missedHighPriorityMsg[id] = 0;
        _lastValidPTZCmd[id] = PTZcmd_;
    }

    void Arbitration::CB_PTZConfigFiltering(rover_msgs::msg::CameraConfig PTZConfig_)
    {
        size_t id = PTZConfig_.id_cam;
        _currentPriorityLevelPtzConfig[id] = _currentPriorityLevelPtzCmd[id];
    }
}  // namespace CameraManager