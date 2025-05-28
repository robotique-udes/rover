#ifndef GNSS_HPP
#define GNSS_HPP

#include "can_master/master_device.hpp"

#include <rover_can2/rover_can2.hpp>
#include "rover_can2/msgs/fix_position.hpp"
#include "rover_can2/msgs/fix_heading.hpp"
#include "rover_can2/msgs/fix_info.hpp"

#include <rover_msgs/msg/gps.hpp>

class Gnss : public RoverCan2::Device<RoverCan2::SubscriberMember<RoverCan2::Msgs::FixHeading, Gnss>,
                                      RoverCan2::SubscriberMember<RoverCan2::Msgs::FixInfo, Gnss>,
                                      RoverCan2::SubscriberMember<RoverCan2::Msgs::FixPosition, Gnss>>,
             public MasterDevice
{
    using DeviceT = RoverCan2::Device<RoverCan2::SubscriberMember<RoverCan2::Msgs::FixHeading, Gnss>,
                                      RoverCan2::SubscriberMember<RoverCan2::Msgs::FixInfo, Gnss>,
                                      RoverCan2::SubscriberMember<RoverCan2::Msgs::FixPosition, Gnss>>;

    static constexpr const char* GNSS_DATA_TOPIC = "/rover/gps/position";
    static constexpr float GNSS_DATA_PUBLISH_FREQUENCY_HZ = 20.0F;

  public:
    Gnss(RoverCan2::Constant::eDeviceId deviceId_);

  private:
    void rosElementInit(void) override;
    void rosElementClean(void) override;
    std::vector<RoverCan2::Constant::eDeviceId> getManagedDevicesIds(void) override;

    void CB_CAN_FixHeading(const RoverCan2::Msgs::FixHeading& canMsg_);
    void CB_CAN_FixInfo(const RoverCan2::Msgs::FixInfo& canMsg_);
    void CB_CAN_FixPosition(const RoverCan2::Msgs::FixPosition& canMsg_);

    rover_msgs::msg::Gps _rosGpsMsg;
    rclcpp::Publisher<rover_msgs::msg::Gps>::SharedPtr _pub_GnssData;
    rclcpp::TimerBase::SharedPtr _gpsPublishTimer;
};

#endif  // GNSS_HPP
