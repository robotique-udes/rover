#include "science.hpp"

#include <rover_lib2/helpers/constants.hpp>
#include <rover_lib2/helpers/macros.hpp>

Science::Science(RoverCan2::Constant::eDeviceId deviceId_):
    DeviceT(deviceId_,
            RoverCan2::Publisher<RoverCan2::Msgs::ScienceCmd>(),
            RoverCan2::SubscriberMember(*this, &Science::CB_CAN_scienceInfo))
{
}

void Science::rosElementInit(void)
{
    _timerCanSend = this->getAttachedNode()->create_wall_timer(std::chrono::milliseconds(CAN_PUBLISH_PERIOD_MS),
                                                               [this](void)
                                                               {
                                                                   this->CB_ROS_canSend();
                                                               });
    _sub_ScienceCmd = this->getAttachedNode()->create_subscription<rover_msgs::msg::ScienceCmd>(
        TOPIC_SCIENCE_CMD,
        QOS_DEFAULT,
        [this](const rover_msgs::msg::ScienceCmd& rosMsg_)
        {
            this->CB_ROS_scienceCmd(rosMsg_);
        });

    _pub_ScienceInfo = this->getAttachedNode()->create_publisher<rover_msgs::msg::ScienceInfo>(TOPIC_SCIENCE_INFO, QOS_DEFAULT);
}

void Science::rosElementClean(void)
{
    if (_timerCanSend)
    {
        _timerCanSend.reset();
    }

    if (_sub_ScienceCmd)
    {
        _sub_ScienceCmd.reset();
    }
}

std::vector<RoverCan2::Constant::eDeviceId> Science::getManagedDevicesIds(void)
{
    return std::vector{this->getCanId()};
}

void Science::CB_ROS_scienceCmd(const rover_msgs::msg::ScienceCmd& rosMsg_)
{
    _nextScienceCmdMsg.data().lin_act_speed = rosMsg_.target_speed[rover_msgs::msg::ScienceCmd::LINEAR_ACT];
    _nextScienceCmdMsg.data().grinder_on = floatToBool(rosMsg_.target_speed[rover_msgs::msg::ScienceCmd::EXCAVATOR]);
    _nextScienceCmdMsg.data().beak_pos = rosMsg_.target_speed[rover_msgs::msg::ScienceCmd::BEAK];
    _nextScienceCmdMsg.data().carrousel_on = floatToBool(rosMsg_.target_speed[rover_msgs::msg::ScienceCmd::CARROUSEL]);
}

void Science::CB_ROS_canSend(void)
{
    this->sendMsg(_nextScienceCmdMsg);
}

void Science::CB_CAN_scienceInfo(const RoverCan2::Msgs::ScienceInfo& canMsg_)
{
    rover_msgs::msg::ScienceInfo rosMsg;

    rosMsg.sample_index = canMsg_.getData().sample_index;
    rosMsg.sensor_1 = canMsg_.getData().sensor_1;
    rosMsg.sensor_2 = canMsg_.getData().sensor_2;
    rosMsg.sensor_3 = canMsg_.getData().sensor_3;
    rosMsg.humidity = canMsg_.getData().humidity;

    _pub_ScienceInfo->publish(rosMsg);
}