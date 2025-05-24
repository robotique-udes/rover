#ifndef LIGHT_HPP
#define LIGHT_HPP

#include "can_master/master_device.hpp"
#include "rover_can2/constant.hpp"
#include "rover_can2/subscriber.hpp"
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rover_can2/rover_can2.hpp>
#include <rover_lib2/helpers/constants.hpp>

#include <rover_can2/msgs/PWM_cmd.hpp>
#include <rover_can2/msgs/PWM_status.hpp>
#include <rover_can2/msgs/PWM_info.hpp>

#include <rover_msgs/msg/detail/light__struct.hpp>
#include <rover_msgs/msg/light.hpp>

class Light : public RoverCan2::Device<RoverCan2::SubscriberMember<RoverCan2::Msgs::PwmStatus, Light>>,
              public MasterDevice
{
    static constexpr const char* TOPIC_LIGHTS_CTRL = "/rover/auxiliary/lights_control";

    using DerivedT = RoverCan2::Device<RoverCan2::SubscriberMember<RoverCan2::Msgs::PwmStatus, Light>>;

  public:
    Light():
        DerivedT(RoverCan2::Constant::eDeviceId::LIGHTS_MAIN,
                 RoverCan2::SubscriberMember<RoverCan2::Msgs::PwmStatus, Light>(*this, &Light::CB_CAN_PwmStatus))
    {
    }

  private:
    void CB_CAN_PwmStatus(const RoverCan2::Msgs::PwmStatus& /*msgCan_*/) {}

    void CB_ROS_lightControl(const rover_msgs::msg::Light& /*msgRos_*/) {}

    void rosElementInit(void)
    {
        sub_lightCmd
            = this->getAttachedNode()->create_subscription<rover_msgs::msg::Light>(TOPIC_LIGHTS_CTRL,
                                                                                   QOS_DEFAULT,
                                                                                   [this](const rover_msgs::msg::Light& msg_)
                                                                                   {
                                                                                       this->CB_ROS_lightControl(msg_);
                                                                                   });
    }

    void rosElementClean(void) {}

    virtual std::vector<RoverCan2::Constant::eDeviceId> getManagedDevicesIds(void) = 0;

    rclcpp::Subscription<rover_msgs::msg::Light>::SharedPtr sub_lightCmd;
    rclcpp::Publisher<rover_msgs::msg::Light>::SharedPtr pub_lightStatus;
};

#endif  // LIGHT_HPP
