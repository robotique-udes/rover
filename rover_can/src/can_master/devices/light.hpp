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

class Light : public RoverCan2::Device<RoverCan2::SubscriberMember<RoverCan2::Msgs::PwmStatus, Light>,
                                       RoverCan2::Publisher<RoverCan2::Msgs::PwmCmd>>,
              public MasterDevice
{
    static constexpr const char* TOPIC_LIGHTS_CTRL = "/rover/auxiliary/lights_control";
    static constexpr const char* TOPIC_LIGHTS_STATUS = "/rover/auxiliary/light_status";
    static constexpr const uint8_t LIGHT_PUBLISH_PERIOD_MS = 100U;

    using DerivedT = RoverCan2::Device<RoverCan2::SubscriberMember<RoverCan2::Msgs::PwmStatus, Light>,
                                       RoverCan2::Publisher<RoverCan2::Msgs::PwmCmd>>;

  public:
    Light(RoverCan2::Constant::eDeviceId deviceId_);

  private:
    void CB_CAN_PwmStatus(const RoverCan2::Msgs::PwmStatus& msgCan_);
    void CB_ROS_lightControl(const rover_msgs::msg::Light& msgRos_);
    void rosElementInit(void);
    void rosElementClean(void);

    std::vector<RoverCan2::Constant::eDeviceId> getManagedDevicesIds(void);

    rclcpp::Subscription<rover_msgs::msg::Light>::SharedPtr _sub_lightCmd;
    rclcpp::Publisher<rover_msgs::msg::Light>::SharedPtr _pub_lightStatus;
    rclcpp::TimerBase::SharedPtr _lightStatusPublishTimer;
    rover_msgs::msg::Light _msgRos;
};

#endif  // LIGHT_HPP
