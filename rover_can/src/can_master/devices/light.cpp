#include "light.hpp"

Light::Light(RoverCan2::Constant::eDeviceId deviceId_):
    DerivedT(deviceId_,
             RoverCan2::SubscriberMember<RoverCan2::Msgs::PwmStatus, Light>(*this, &Light::CB_CAN_PwmStatus),
             RoverCan2::Publisher<RoverCan2::Msgs::PwmCmd>())
{
}

void Light::CB_CAN_PwmStatus(const RoverCan2::Msgs::PwmStatus& msgCan_)
{
    _msgRos.duty_cycle = msgCan_.getData().dutyCycle;
    _msgRos.frequency = msgCan_.getData().frequency;
}

void Light::CB_ROS_lightControl(const rover_msgs::msg::Light& msgRos_)
{
    RoverCan2::Msgs::PwmCmd msgCan;
    msgCan.data().dutyCycle = msgRos_.duty_cycle;
    msgCan.data().frequency = msgRos_.frequency;
    this->sendMsg(msgCan);
}

void Light::rosElementInit(void)
{
    _sub_lightCmd
        = this->getAttachedNode()->create_subscription<rover_msgs::msg::Light>(TOPIC_LIGHTS_CTRL,
                                                                               QOS_DEFAULT,
                                                                               [this](const rover_msgs::msg::Light& msg_)
                                                                               {
                                                                                   this->CB_ROS_lightControl(msg_);
                                                                               });
    _pub_lightStatus = this->getAttachedNode()->create_publisher<rover_msgs::msg::Light>(TOPIC_LIGHTS_STATUS, QOS_DEFAULT);

    _lightStatusPublishTimer = this->getAttachedNode()->create_wall_timer(std::chrono::milliseconds(LIGHT_PUBLISH_PERIOD_MS),
                                                                          [this]()
                                                                          {
                                                                              if (_pub_lightStatus)
                                                                              {
                                                                                  _pub_lightStatus->publish(_msgRos);
                                                                              }
                                                                          });
}

void Light::rosElementClean(void)
{
    if (_sub_lightCmd)
    {
        _sub_lightCmd.reset();
    }

    if (_pub_lightStatus)
    {
        _pub_lightStatus.reset();
    }
}

std::vector<RoverCan2::Constant::eDeviceId> Light::getManagedDevicesIds(void)
{
    return std::vector{this->getCanId()};
}
