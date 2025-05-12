#include "gnss.hpp"
#include "rover_can2/msgs/fix_info.hpp"
#include "rover_can2/msgs/fix_position.hpp"

Gnss::Gnss(RoverCan2::Constant::eDeviceId deviceId_):
    DeviceT(deviceId_,
            RoverCan2::SubscriberMember(*this, &Gnss::CB_CAN_FixHeading),
            RoverCan2::SubscriberMember(*this, &Gnss::CB_CAN_FixInfo),
            RoverCan2::SubscriberMember(*this, &Gnss::CB_CAN_FixPosition))
{
}

void Gnss::rosElementInit(void)
{
    _pub_GnssData = this->getAttachedNode()->create_publisher<rover_msgs::msg::Gps>(GNSS_DATA_TOPIC, 1);
}

void Gnss::rosElementClean(void)
{
    if (_pub_GnssData)
    {
        _pub_GnssData.reset();
    }
}

std::vector<RoverCan2::Constant::eDeviceId> Gnss::getManagedDevicesIds(void)
{
    return std::vector{this->getCanId()};
}

void Gnss::CB_CAN_FixHeading(const RoverCan2::Msgs::FixHeading& canMsg_)
{
    
}

void Gnss::CB_CAN_FixInfo(const RoverCan2::Msgs::FixInfo& canMsg_)
{
    
}

void Gnss::CB_CAN_FixPosition(const RoverCan2::Msgs::FixPosition& canMsg_)
{

}