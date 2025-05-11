#ifndef GNSS_HPP
#define GNSS_HPP

#include "can_master/master_device.hpp"
#include "can_master/shared_msg.hpp"
#include "rover_can2/msgs/fix_position.hpp"
#include "rover_can2/msgs/fix_heading.hpp"
#include "rover_can2/msgs/fix_info.hpp"
#include "rover_can2/msgs/fix_position.hpp"
#include "rover_can2/subscriber.hpp"

#include <rover_can2/rover_can2.hpp>

class Gnss : public RoverCan2::Device<RoverCan2::SubscriberMember<RoverCan2::Msgs::FixHeading, Gnss>,
                                      RoverCan2::SubscriberMember<RoverCan2::Msgs::FixInfo, Gnss>,
                                      RoverCan2::SubscriberMember<RoverCan2::Msgs::FixPosition, Gnss>>,
             public MasterDevice
{
    using DeviceT = RoverCan2::Device<RoverCan2::SubscriberMember<RoverCan2::Msgs::FixHeading, Gnss>,
                                      RoverCan2::SubscriberMember<RoverCan2::Msgs::FixInfo, Gnss>,
                                      RoverCan2::SubscriberMember<RoverCan2::Msgs::FixPosition, Gnss>>;
};

#endif  // GNSS_HPP