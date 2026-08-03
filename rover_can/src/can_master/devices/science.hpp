#ifndef SCIENCE_HPP
#define SCIENCE_HPP

#include "can_master/master_device.hpp"
#include "can_master/shared_msg.hpp"

#include <rover_can2/msgs/science_cmd.hpp>
#include <rover_msgs/msg/science_msg.hpp>

#include <rover_can2/rover_can2.hpp>

class Science : public RoverCan2::Device<RoverCan2::Publisher<RoverCan2::Msgs::ScienceCmd>>,
                public MasterDevice
{
    using DeviceT = RoverCan2::Device<RoverCan2::Publisher<RoverCan2::Msgs::ScienceCmd>>;

    static constexpr const char* SCIENCE_CMD_TOPIC = "/rover/science/cmd";

    static constexpr float CAN_PUBLISH_FREQUENCY = 20.0F;
    static constexpr uint32_t CAN_PUBLISH_PERIOD_MS = static_cast<uint32_t>(ROUND(1'000.0F / CAN_PUBLISH_FREQUENCY));

  public:
    Science(RoverCan2::Constant::eDeviceId deviceId_);

  private:
    void rosElementInit(void) override;
    void rosElementClean(void) override;
    std::vector<RoverCan2::Constant::eDeviceId> getManagedDevicesIds(void) override;

    void CB_ROS_scienceCmd(const rover_msgs::msg::ScienceMsg& rosMsg_);
    void CB_ROS_canSend(void);


    std::shared_ptr<CanMaster::SharedRosMsg<rover_msgs::msg::ScienceMsg>> _rosSharedMsg;
    RoverCan2::Msgs::ScienceCmd _nextScienceCmdMsg;

    rclcpp::Subscription<rover_msgs::msg::ScienceMsg>::SharedPtr _sub_MotorStatus;
    rclcpp::TimerBase::SharedPtr _timerCanSend;
};

#endif  // PROPULSIONMOTORS_HPP
