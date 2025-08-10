#ifndef SENSOR_BOX_HPP
#define SENSOR_BOX_HPP

#include "can_master/master_device.hpp"

#include <rclcpp/timer.hpp>
#include <rover_can2/subscriber.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rover_can2/rover_can2.hpp>
#include <rover_lib2/helpers/constants.hpp>

#include <rover_can2/msgs/sensor_box.hpp>
#include <rover_msgs/msg/sensor_box.hpp>

class SensorBox : public RoverCan2::Device<RoverCan2::SubscriberMember<RoverCan2::Msgs::SensorBox, SensorBox>>,
                  public MasterDevice
{
    static constexpr const char* TOPIC_VALUES = "/rover/auxiliary/sensor_box";
    static constexpr const uint8_t LIGHT_PUBLISH_PERIOD_MS = 100U;

    using DerivedT = RoverCan2::Device<RoverCan2::SubscriberMember<RoverCan2::Msgs::SensorBox, SensorBox>>;

  public:
    explicit SensorBox();
    virtual ~SensorBox() = default;

  private:
    void CB_CAN_SensorBoxValues(const RoverCan2::Msgs::SensorBox& msgCan_);
    void rosElementInit() override;
    void rosElementClean() override;

    std::vector<RoverCan2::Constant::eDeviceId> getManagedDevicesIds() override;

    rclcpp::Publisher<rover_msgs::msg::SensorBox>::SharedPtr _pub_sensorBox;
    rclcpp::TimerBase::SharedPtr _timer_publisher;
    rover_msgs::msg::SensorBox _msgRos;
};

#endif  // SENSOR_BOX_HPP
