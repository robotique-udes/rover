#include "camera.hpp"
#include "rover_can2/msgs/PTZ_cmd.hpp"
#include "rover_can2/msgs/PTZ_config.hpp"
#include "rover_can2/msgs/PTZ_status.hpp"
#include "rover_can2/msgs/power_cmd.hpp"
#include <rover_lib2/helpers/constants.hpp>
#include <rover_msgs/msg/detail/camera_config__struct.hpp>

Camera::Camera(RoverCan2::Constant::eDeviceId IdCan_, uint8_t IdCameraControlMsgCam_):
    Device(IdCan_,
           RoverCan2::Publisher<RoverCan2::Msgs::PowerCmd, 1>(),
           RoverCan2::Publisher<RoverCan2::Msgs::PtzCmd, 1>(),
           RoverCan2::Publisher<RoverCan2::Msgs::PtzConfig, 1>(),
           RoverCan2::SubscriberMember<RoverCan2::Msgs::PowerStatus, Camera>(*this, &Camera::CB_CAN_powerStatus),
           RoverCan2::SubscriberMember<RoverCan2::Msgs::PtzStatus, Camera>(*this, &Camera::CB_CAN_ptzStatus)),

    _idCamCameraControlMsg(IdCameraControlMsgCam_)
{
}

void Camera::rosElementInit()
{
    _pub_powerStatus
        = this->getAttachedNode()->create_publisher<rover_msgs::msg::CameraControl>(TOPIC_CAMERA_POWER_STATUS, QOS_DEFAULT);

    _pub_ptzStatus
        = this->getAttachedNode()->create_publisher<rover_msgs::msg::CameraControl>(TOPIC_CAMERA_PTZ_STATUS, QOS_DEFAULT);

    _sub_powerCmd = this->getAttachedNode()->create_subscription<rover_msgs::msg::CameraControl>(
        TOPIC_CAMERA_POWER_COMMAND,
        QOS_DEFAULT,
        [this](const rover_msgs::msg::CameraControl& rosMsg_)
        {
            this->CB_ROS_powerCmd(rosMsg_);
        });

    _sub_ptzCmd = this->getAttachedNode()->create_subscription<rover_msgs::msg::CameraControl>(
        TOPIC_CAMERA_PTZ_COMMAND_MANAGER,
        QOS_DEFAULT,
        [this](const rover_msgs::msg::CameraControl& rosMsg_)
        {
            this->CB_ROS_ptzCmd(rosMsg_);
        });

    _sub_ptzConfig = this->getAttachedNode()->create_subscription<rover_msgs::msg::CameraConfig>(
        TOPIC_CAMERA_PTZ_CONFIG_MANAGER,
        QOS_DEFAULT,
        [this](const rover_msgs::msg::CameraConfig& rosMsg_)
        {
            this->CB_ROS_ptzConfig(rosMsg_);
        });
}

void Camera::rosElementClean()
{
    if (_pub_powerStatus)
    {
        _pub_powerStatus.reset();
    }

    if (_pub_ptzStatus)
    {
        _pub_ptzStatus.reset();
    }

    if (_sub_powerCmd)
    {
        _sub_powerCmd.reset();
    }

    if (_sub_ptzCmd)
    {
        _sub_ptzCmd.reset();
    }

    if (_sub_ptzConfig)
    {
        _sub_ptzConfig.reset();
    }

    if (_timerCanSendPowerCmd)
    {
        _timerCanSendPowerCmd.reset();
    }

    if (_timerCanSendPtzCmd)
    {
        _timerCanSendPtzCmd.reset();
    }

    if (_timerCanSendPtzConfig)
    {
        _timerCanSendPtzConfig.reset();
    }
}

std::vector<RoverCan2::Constant::eDeviceId> Camera::getManagedDevicesIds(void)
{
    return std::vector{this->getCanId()};
}

void Camera::CB_CAN_powerStatus(const RoverCan2::Msgs::PowerStatus& canMsg_)
{
    rover_msgs::msg::CameraControl rosMsg;
    rosMsg.id_cam = _idCamCameraControlMsg;
    rosMsg.power_on = canMsg_.getData().on_state;
    _pub_powerStatus->publish(rosMsg);
}

void Camera::CB_CAN_ptzStatus(const RoverCan2::Msgs::PtzStatus& canMsg_)
{
    rover_msgs::msg::CameraControl rosMsg;
    rosMsg.id_cam = _idCamCameraControlMsg;
    rosMsg.pitch = canMsg_.getData().tilt;
    rosMsg.yaw = canMsg_.getData().pan;
    _pub_ptzStatus->publish(rosMsg);
}

void Camera::CB_ROS_powerCmd(const rover_msgs::msg::CameraControl& rosMsg_)
{
    if (rosMsg_.id_cam != _idCamCameraControlMsg)
    {
        // Not concerned
        return;
    }

    _nextPowerCanMsg.data().onState = rosMsg_.power_on;

    if (!_timerCanSendPowerCmd && this->getAttachedNode())
    {
        _timerCanSendPowerCmd = this->getAttachedNode()->create_wall_timer(
            std::chrono::milliseconds(static_cast<size_t>(1000 / CAMERA_POWER_STATUS_PUB_FREQ)),
            [this](void)
            {
                this->CB_sendCanPowerCmd();
            });
    }
}

void Camera::CB_ROS_ptzCmd(const rover_msgs::msg::CameraControl& rosMsg_)
{
    if (rosMsg_.id_cam != _idCamCameraControlMsg)
    {
        // Not concerned
        return;
    }

    _nextPtzCmd.data().pan = rosMsg_.yaw;
    _nextPtzCmd.data().tilt = rosMsg_.pitch;

    if (!_timerCanSendPtzCmd && this->getAttachedNode())
    {
        _timerCanSendPtzCmd = this->getAttachedNode()->create_wall_timer(
            std::chrono::milliseconds(static_cast<size_t>(1000 / CAMERA_PTZ_CMD_PUB_FREQ)),
            [this](void)
            {
                this->CB_sendCanPtzCmd();
            });
    }
}

void Camera::CB_ROS_ptzConfig(const rover_msgs::msg::CameraConfig& rosMsg_)
{
    if (rosMsg_.id_cam != _idCamCameraControlMsg)
    {
        // Not concerned
        return;
    }

    _nextPtzConfig.data().panMaxPosition = rosMsg_.pan_max_position;
    _nextPtzConfig.data().panMinPosition = rosMsg_.pan_min_position;
    _nextPtzConfig.data().panMaxSpeed = rosMsg_.pan_max_speed;

    _nextPtzConfig.data().tiltMaxPosition = rosMsg_.tilt_max_position;
    _nextPtzConfig.data().tiltMinPosition = rosMsg_.tilt_min_position;
    _nextPtzConfig.data().tiltMaxSpeed = rosMsg_.tilt_max_speed;

    if (!_timerCanSendPtzConfig && this->getAttachedNode())
    {
        _timerCanSendPtzConfig = this->getAttachedNode()->create_wall_timer(
            std::chrono::milliseconds(static_cast<size_t>(1000 / CAMERA_PTZ_CONFIG_PUB_FREQ)),
            [this](void)
            {
                this->CB_sendCanPtzConfig();
            });
    }
}

void Camera::CB_sendCanPowerCmd(void)
{
    this->sendMsg(_nextPowerCanMsg);
}

void Camera::CB_sendCanPtzCmd(void)
{
    this->sendMsg(_nextPtzCmd);
}

void Camera::CB_sendCanPtzConfig(void)
{
    this->sendMsg(_nextPtzConfig);
}