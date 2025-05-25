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

    auto period = std::chrono::duration<double>(1.0 / GNSS_DATA_PUBLISH_FREQUENCY_HZ);
    _gpsPublishTimer = this->getAttachedNode()->create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(period),
                                                                  [this]()
                                                                  {
                                                                      if (_pub_GnssData && _rosGpsMsg.fix_quality != 0)
                                                                      {
                                                                          _pub_GnssData->publish(_rosGpsMsg);
                                                                      }
                                                                  });
}

void Gnss::rosElementClean(void)
{
    if (_pub_GnssData)
    {
        _pub_GnssData.reset();
    }

    if (_gpsPublishTimer)
    {
        _gpsPublishTimer.reset();
    }
}

std::vector<RoverCan2::Constant::eDeviceId> Gnss::getManagedDevicesIds(void)
{
    return std::vector{this->getCanId()};
}

void Gnss::CB_CAN_FixHeading(const RoverCan2::Msgs::FixHeading& canMsg_)
{
    float rawHeading = canMsg_.getData().headingDeg;
    float calibratedHeading = std::fmod(rawHeading + HEADING_CALIB_VALUE, 360.0F);
    while (calibratedHeading < 0.0F)
    {
        calibratedHeading += 360.0F;
    }

    while (calibratedHeading > 360.0F)
    {
        calibratedHeading -= 360.0F;
    }

    _rosGpsMsg.heading = calibratedHeading;
}

void Gnss::CB_CAN_FixInfo(const RoverCan2::Msgs::FixInfo& canMsg_)
{
    _rosGpsMsg.satellite = canMsg_.getData().satelliteCount;
    _rosGpsMsg.fix_quality = canMsg_.getData().fixQuality;
}

void Gnss::CB_CAN_FixPosition(const RoverCan2::Msgs::FixPosition& canMsg_)
{
    _rosGpsMsg.latitude = canMsg_.getData().latitude;
    _rosGpsMsg.longitude = canMsg_.getData().longitude;
}
