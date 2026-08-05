#include "science.hpp"

#include <rover_can2/msgs/science_cmd.hpp>
#include <rover_lib2/helpers/constants.hpp>
#include <rover_lib2/helpers/macros.hpp>

Science::Science(RoverCan2::Constant::eDeviceId deviceId_):
    DeviceT(deviceId_, RoverCan2::Publisher<RoverCan2::Msgs::ScienceCmd>())
{
}

void Science::rosElementInit(void)
{
    _timerCanSend = this->getAttachedNode()->create_wall_timer(std::chrono::milliseconds(CAN_PUBLISH_PERIOD_MS),
                                                               [this](void)
                                                               {
                                                                   this->CB_ROS_canSend();
                                                               });
    _sub_MotorStatus = this->getAttachedNode()->create_subscription<rover_msgs::msg::ScienceMsg>(
        SCIENCE_CMD_TOPIC,
        QOS_DEFAULT,
        [this](const rover_msgs::msg::ScienceMsg& rosMsg_)
        {
            this->CB_ROS_scienceCmd(rosMsg_);
        });
}

void Science::rosElementClean(void)
{
    if (_timerCanSend)
    {
        _timerCanSend.reset();
    }

    if (_sub_MotorStatus)
    {
        _sub_MotorStatus.reset();
    }
}

std::vector<RoverCan2::Constant::eDeviceId> Science::getManagedDevicesIds(void)
{
    return std::vector{this->getCanId()};
}

void Science::CB_ROS_scienceCmd(const rover_msgs::msg::ScienceMsg& rosMsg_)
{
    _nextScienceCmdMsg.data().lin_act_speed = rosMsg_.target_speed[rover_msgs::msg::ScienceMsg::LINEAR_ACT];
    _nextScienceCmdMsg.data().grinder_on = floatToBool(rosMsg_.target_speed[rover_msgs::msg::ScienceMsg::EXCAVATOR]);
    _nextScienceCmdMsg.data().beak_pos = rosMsg_.target_speed[rover_msgs::msg::ScienceMsg::BEAK];
    _nextScienceCmdMsg.data().carrousel_on = floatToBool(rosMsg_.target_speed[rover_msgs::msg::ScienceMsg::CARROUSEL]);
}

void Science::CB_ROS_canSend(void)
{
    this->sendMsg(_nextScienceCmdMsg);
}
