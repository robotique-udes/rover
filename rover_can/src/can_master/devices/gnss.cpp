#include "gnss.hpp"
#include "rover_can2/msgs/fix_info.hpp"
#include "rover_can2/msgs/fix_position.hpp"

Gnss::Gnss(RoverCan2::Constant::eDeviceId deviceId_,
           std::shared_ptr<CanMaster::SharedRosMsg<rover_msgs::msg::Gps>> rosSharedMsg_):
    DeviceT(deviceId_,
            RoverCan2::SubscriberMember(*this, &Gnss::CB_CAN_FixHeading),
            RoverCan2::SubscriberMember(*this, &Gnss::CB_CAN_FixInfo),
            RoverCan2::SubscriberMember(*this, &Gnss::CB_CAN_FixPosition)),
    _rosSharedMsg(rosSharedMsg_)
{
    ASSERT_COND_MSG(rosSharedMsg_, "rosSharedMsg_ can't be nullptr");
}

void Gnss::rosElementInit(void)
{
    _pub_GnssData = this->getAttachedNode()->create_publisher<rover_msgs::msg::Gps>(GNSS_DATA_TOPIC, 1);
    _rosSharedMsg->attachNewPub(this->getAttachedNode(), _pub_GnssData, GNSS_DATA_PUBLISH_FREQUENCY_HZ);
}

void Gnss::rosElementClean(void)
{
    if (_pub_GnssData)
    {
        _rosSharedMsg->removePub(_pub_GnssData);
        _pub_GnssData.reset();
    }
}

std::vector<RoverCan2::Constant::eDeviceId> Gnss::getManagedDevicesIds(void)
{
    return std::vector{this->getCanId()};
}

void Gnss::CB_CAN_FixHeading(const RoverCan2::Msgs::FixHeading& canMsg_)
{
    _rosSharedMsg->get().getThreadSafeAccess().heading = canMsg_.getData().headingDeg;
}

void Gnss::CB_CAN_FixInfo(const RoverCan2::Msgs::FixInfo& canMsg_)
{
    _rosSharedMsg->get().getThreadSafeAccess().fix_quality = canMsg_.getData().fixQuality;
    _rosSharedMsg->get().getThreadSafeAccess().satellite = canMsg_.getData().satelliteCount;
}

void Gnss::CB_CAN_FixPosition(const RoverCan2::Msgs::FixPosition& canMsg_)
{
    _rosSharedMsg->get().getThreadSafeAccess().latitude = canMsg_.getData().latitude;
    _rosSharedMsg->get().getThreadSafeAccess().longitude = canMsg_.getData().longitude;
}