#include "CameraMain.hpp"

CameraMain::CameraMain():
    Device(RoverCan2::Constant::eDeviceId::CAMERA_ROVER_MAIN,
           RoverCan2::Publisher<RoverCan2::Msgs::PowerCmd, 1>(),
           RoverCan2::SubscriberMember<RoverCan2::Msgs::PowerStatus, CameraMain>(*this, &CameraMain::CB_CAN_powerStatus))
{
}

void CameraMain::rosElementInit()
{
    _pub_powerStatus = _rosNode->create_publisher<rover_msgs::msg::DevicePower>("/rover/device/camera_main/power_status", 1);
    _sub_powerStatus
        = _rosNode->create_subscription<rover_msgs::msg::DevicePower>("/rover/device/camera_main/power_cmd",
                                                                      1,
                                                                      [this](const rover_msgs::msg::DevicePower& rosMsg_)
                                                                      {
                                                                          this->CB_ROS_powerCmd(rosMsg_);
                                                                      });
}

void CameraMain::rosElementClean()
{
    if (_pub_powerStatus)
    {
        _pub_powerStatus.reset();
    }

    if (_sub_powerStatus)
    {
        _sub_powerStatus.reset();
    }

    if (_timerCanSendPowerCmd)
    {
        _timerCanSendPowerCmd.reset();
    }
}

void CameraMain::CB_CAN_powerStatus(const RoverCan2::Msgs::PowerStatus& canMsg_)
{
    rover_msgs::msg::DevicePower rosMsg;
    rosMsg.on = canMsg_.getData().on_state;
    _pub_powerStatus->publish(rosMsg);
}

void CameraMain::CB_ROS_powerCmd(const rover_msgs::msg::DevicePower& rosMsg_)
{
    _nextCanMsg.data().onState = rosMsg_.on;

    if (!_timerCanSendPowerCmd && _rosNode)
    {
        _rosNode->create_wall_timer(std::chrono::milliseconds(500),
                                    [this](void)
                                    {
                                        this->CB_sendCanPowerCmd();
                                    });
    }
}

void CameraMain::CB_sendCanPowerCmd(void)
{
    this->sendMsg(_nextCanMsg);
}
