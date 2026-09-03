#ifndef SCIENCE_HPP
#define SCIENCE_HPP

#include "can_master/master_device.hpp"
#include "can_master/shared_msg.hpp"

#include <rover_can2/msgs/science_cmd.hpp>
#include <rover_can2/msgs/science_info.hpp>
#include <rover_msgs/msg/science_cmd.hpp>
#include <rover_msgs/msg/science_info.hpp>

#include <rover_can2/rover_can2.hpp>

class Science : public RoverCan2::Device<RoverCan2::Publisher<RoverCan2::Msgs::ScienceCmd>,
                                         RoverCan2::SubscriberMember<RoverCan2::Msgs::ScienceInfo, Science>>,
                public MasterDevice
{
    using DeviceT = RoverCan2::Device<RoverCan2::Publisher<RoverCan2::Msgs::ScienceCmd>,
                                      RoverCan2::SubscriberMember<RoverCan2::Msgs::ScienceInfo, Science>>;

    static constexpr const char* TOPIC_SCIENCE_CMD = "/rover/science/cmd";
    static constexpr const char* TOPIC_SCIENCE_INFO = "/rover/science/info";

    static constexpr float CAN_PUBLISH_FREQUENCY = 20.0F;
    static constexpr uint32_t CAN_PUBLISH_PERIOD_MS = static_cast<uint32_t>(ROUND(1'000.0F / CAN_PUBLISH_FREQUENCY));

  public:
    Science(RoverCan2::Constant::eDeviceId deviceId_);

  private:
    void rosElementInit(void) override;
    void rosElementClean(void) override;
    std::vector<RoverCan2::Constant::eDeviceId> getManagedDevicesIds(void) override;

    void CB_ROS_scienceCmd(const rover_msgs::msg::ScienceCmd& rosMsg_);
    void CB_ROS_canSend(void);
    void CB_CAN_scienceInfo(const RoverCan2::Msgs::ScienceInfo& canMsg_);

    RoverCan2::Msgs::ScienceCmd _nextScienceCmdMsg;

    rclcpp::Subscription<rover_msgs::msg::ScienceCmd>::SharedPtr _sub_ScienceCmd;
    rclcpp::TimerBase::SharedPtr _timerCanSend;

    rclcpp::Publisher<rover_msgs::msg::ScienceInfo>::SharedPtr _pub_ScienceInfo;
};

#endif  // PROPULSIONMOTORS_HPP
