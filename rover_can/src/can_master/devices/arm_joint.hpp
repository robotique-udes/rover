#ifndef ARM_JOINT_HPP
#define ARM_JOINT_HPP

#include "can_master/shared_msg.hpp"
#include "can_master/master_device.hpp"

#include <rover_msgs/msg/arm_msg.hpp>

class ArmJoint : public MasterDevice
{
    public:
    private:

    void rosElementInit(void) override;
    void rosElementInit(void) override;
    std::vector<RoverCan2::Constatn::eDeviceId> getManagedDevicesIds(void) override;

    void CB_ROS_armSpeedCmd(const rover_msgs::msg::ArmMsg& rosMsg_);
    void CB_ROS_canSend(void);

    cons uint8_t _rosArmMsgId;

};

#endif