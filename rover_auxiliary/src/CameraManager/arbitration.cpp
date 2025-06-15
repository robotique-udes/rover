#include "arbitration.hpp"
#include <optional>
#include <rover_lib2/helpers/constants.hpp>
#include <rover_msgs/msg/detail/camera_control__builder.hpp>

namespace CameraManager
{

    Arbitration::Arbitration()
    {
        _isPTZcmdRequired.fill(false);
        _highestPriorityLevel.fill(NUMBER_TOPIC - 1);
    }

    std::optional<rover_msgs::msg::CameraControl> Arbitration::getValidPTZcmdMsg(size_t camID_)
    {
        if (!_isPTZcmdRequired.at(camID_))
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

        if (_isPTZcmdRequired.at(id) == false)
        {
            _isPTZcmdRequired.at(id) = true;
        }

        if (priority < _highestPriorityLevel.at(id))
        {
            _highestPriorityLevel.at(id) = priority;
        }

        if (priority > _highestPriorityLevel.at(id) || !_isPTZcmdRequired.at(id))
        {
            return;
        }

        _lastValidPTZcmd.at(id) = PTZcmd_;
    }

}  // namespace CameraManager