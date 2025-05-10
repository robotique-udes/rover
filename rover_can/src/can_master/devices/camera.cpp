#include "camera.hpp"

Camera::Camera(RoverCan2::Constant::eDeviceId IdCan_, uint8_t IdCameraControlMsgCam_):
    Device(IdCan_,
           RoverCan2::Publisher<RoverCan2::Msgs::PowerCmd, 1>(),
           RoverCan2::SubscriberMember<RoverCan2::Msgs::PowerStatus, Camera>(*this, &Camera::CB_CAN_powerStatus)),
    _idCamCameraControlMsg(IdCameraControlMsgCam_)
{
}

void Camera::rosElementInit()
{
    _pub_powerStatus = this->getAttachedNode()->create_publisher<rover_msgs::msg::CameraControl>(CAMERA_POWER_STATUS_TOPIC, QOS_DEFAULT);
    _sub_powerCmd = this->getAttachedNode()->create_subscription<rover_msgs::msg::CameraControl>(
        CAMERA_POWER_CONTROL_TOPIC,
        QOS_DEFAULT,
        [this](const rover_msgs::msg::CameraControl& rosMsg_)
        {
            this->CB_ROS_powerCmd(rosMsg_);
        });
}

void Camera::rosElementClean()
{
    if (_pub_powerStatus)
    {
        _pub_powerStatus.reset();
    }

    if (_sub_powerCmd)
    {
        _sub_powerCmd.reset();
    }

    if (_timerCanSendPowerCmd)
    {
        _timerCanSendPowerCmd.reset();
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

void Camera::CB_ROS_powerCmd(const rover_msgs::msg::CameraControl& rosMsg_)
{
    if (rosMsg_.id_cam != _idCamCameraControlMsg)
    {
        // Not concerned
        return;
    }

    _nextCanMsg.data().onState = rosMsg_.power_on;

    if (!_timerCanSendPowerCmd && this->getAttachedNode())
    {
        _timerCanSendPowerCmd
            = this->getAttachedNode()->create_wall_timer(std::chrono::milliseconds(CAMERA_POWER_STATUS_PUB_PERIOD_MS),
                                                         [this](void)
                                                         {
                                                             this->CB_sendCanPowerCmd();
                                                         });
    }
}

void Camera::CB_sendCanPowerCmd(void)
{
    this->sendMsg(_nextCanMsg);
}
