#ifndef ARM_JOINT_HPP
#define ARM_JOINT_HPP

#include <rover_can2/publisher.hpp>
#include <rover_can2/subscriber.hpp>
#include <rover_can2/device.hpp>

#include "can_master/shared_msg.hpp"
#include "can_master/master_device.hpp"

#include <rover_msgs/msg/arm_msg.hpp>
#include <rover_can2/msgs/arm_speed_cmd.hpp>



class ArmJoint : public RoverCan2::Device
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