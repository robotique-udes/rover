#include "gnss.hpp"

Gnss::Gnss(RoverCan2::Constant::eDeviceId deviceId_,
           uint8_t rosGnssDataMsgId_,
           std::shared_ptr<CanMaster::SharedRosMsg<rover_msgs::msg::Gps>> rosSharedMsg_):
    DeviceT(deviceId_,
            RoverCan2::SubscriberMember(*this, &Gnss::CB_CAN_FixHeading),
            RoverCan2::SubscriberMember(*this, &Gnss::CB_CAN_FixInfo),
            RoverCan2::SubscriberMember(*this, &Gnss::CB_CAN_FixPosition)),
    _rosGnssDataMsgId(rosGnssDataMsgId_),
    _rosSharedMsg(rosSharedMsg_)
{
    ASSERT_COND_MSG(rosSharedMsg_, "rosSharedMsg_ can't be nullptr");
}