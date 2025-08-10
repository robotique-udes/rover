#include "sensor_box.hpp"

#include "rover_can2/constant.hpp"

SensorBox::SensorBox():
    DerivedT(RoverCan2::Constant::eDeviceId::GAS_SENSORS,
             RoverCan2::SubscriberMember<RoverCan2::Msgs::SensorBox, SensorBox>(*this, &SensorBox::CB_CAN_SensorBoxValues))
{
}

void SensorBox::CB_CAN_SensorBoxValues(const RoverCan2::Msgs::SensorBox& msgCan_)
{
    _msgRos.amonia_ppb = msgCan_.getData().amonia;
    _msgRos.hydrogen_percent = msgCan_.getData().hydrogen;
}

void SensorBox::rosElementInit(void)
{
    _pub_sensorBox = this->getAttachedNode()->create_publisher<rover_msgs::msg::SensorBox>(TOPIC_VALUES, QOS_DEFAULT);

    _timer_publisher = this->getAttachedNode()->create_wall_timer(std::chrono::milliseconds(LIGHT_PUBLISH_PERIOD_MS),
                                                                  [this]()
                                                                  {
                                                                      if (_pub_sensorBox)
                                                                      {
                                                                          _pub_sensorBox->publish(_msgRos);
                                                                      }
                                                                  });
}

void SensorBox::rosElementClean(void)
{
    if (_pub_sensorBox)
    {
        _pub_sensorBox.reset();
    }

    if (_timer_publisher)
    {
        _timer_publisher.reset();
    }
}

std::vector<RoverCan2::Constant::eDeviceId> SensorBox::getManagedDevicesIds(void)
{
    return std::vector{this->getCanId()};
}
