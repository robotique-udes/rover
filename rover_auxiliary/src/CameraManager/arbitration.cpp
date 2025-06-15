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
        _highestPriorityLevel.fill(NUMBER_TOPIC - 1);
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
            return _lastValidPTZcmd.at(camID_);
        }
    }

    void Arbitration::CB_PTZcmdFiltering(rover_msgs::msg::CameraControl PTZcmd_, size_t priority)
    {
        size_t id = PTZcmd_.id_cam;

        if (_isPTZTopicActive.at(id) == false)
        {
            _isPTZTopicActive.at(id) = true;
            _highestPriorityLevel.at(id) = priority;
            RCLCPP_INFO(rclcpp::get_logger("CAMERA_ARBITRATION"), "Cam %ld is now managed by %s", id, PTZ_CMD_TOPIC[priority]);
            _activeTopicCount++;
        }

        else if (priority < _highestPriorityLevel.at(id))
        {
            _highestPriorityLevel.at(id) = priority;
            RCLCPP_INFO(rclcpp::get_logger("CAMERA_ARBITRATION"),
                        "Priority has shifted up to %s on cam %ld",
                        PTZ_CMD_TOPIC[priority],
                        id);
        }

        if (priority > _highestPriorityLevel.at(id) || !_isPTZTopicActive.at(id))
        {
            _missingHighPriorityMsg.at(id)++;

            size_t highPriorityMissingMsgThreshold = _activeTopicCount + HIGH_PRIORITY_MISSING_MSG_SAFETY_FACTOR;
            RCLCPP_INFO(rclcpp::get_logger("CAMERA_ARBITRATION"), "THRESH %ld ", highPriorityMissingMsgThreshold);

            if (_missingHighPriorityMsg.at(id) > highPriorityMissingMsgThreshold)
            {
                
                _highestPriorityLevel.at(id) = priority;
            RCLCPP_INFO(rclcpp::get_logger("CAMERA_ARBITRATION"),
                        "Priority has shifted down to %s on cam %ld",
                        PTZ_CMD_TOPIC[priority],
                        id);
            }

            #warning watchdog for the least prioritize topic

            return;
        }

        _missingHighPriorityMsg.at(id) = 0;
        _lastValidPTZcmd.at(id) = PTZcmd_;
    }

}  // namespace CameraManager