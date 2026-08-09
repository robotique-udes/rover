#ifndef MORSE_INPUT_HPP
#define MORSE_INPUT_HPP

#include <vector>

#include "can_master/master_device.hpp"

#include <rover_can2/rover_can2.hpp>
#include <rover_can2/msgs/morse_code.hpp>
#include <rover_can2/msgs/morse_status.hpp>
#include <rover_msgs/msg/morse_code.hpp>
#include <rover_msgs/msg/morse_status.hpp>
#include <mutex>

class MorseInput : public RoverCan2::Device<RoverCan2::Publisher<RoverCan2::Msgs::MorseCode>,
                                            RoverCan2::SubscriberMember<RoverCan2::Msgs::MorseStatus, MorseInput>>,
                   public MasterDevice
{
    static constexpr const char* TOPIC_MORSE_CODE = "/base/gui/morse_code";
    static constexpr const char* TOPIC_MORSE_STATUS = "/base/gui/morse_status";
    static constexpr const uint8_t STATUS_PUBLISH_PERIOD_MS = 100U;

    using DerivedT = RoverCan2::Device<RoverCan2::Publisher<RoverCan2::Msgs::MorseCode>,
                                       RoverCan2::SubscriberMember<RoverCan2::Msgs::MorseStatus, MorseInput>>;

  public:
    MorseInput(RoverCan2::Constant::eDeviceId IdCan_);

  private:
    void rosElementInit() override;
    void rosElementClean() override;
    std::vector<RoverCan2::Constant::eDeviceId> getManagedDevicesIds(void) override;

    void CB_ROS_morseCode(const rover_msgs::msg::MorseCode& rosMsg_);
    void CB_CAN_MorseStatus(const RoverCan2::Msgs::MorseStatus& msgCan_);

    RoverCan2::Msgs::MorseCode _nextMorseInputMsg;

    rclcpp::Subscription<rover_msgs::msg::MorseCode>::SharedPtr _sub_morseCode;
    rclcpp::Publisher<rover_msgs::msg::MorseStatus>::SharedPtr _pub_morseStatus;
    rclcpp::TimerBase::SharedPtr _timer_statusPublisher;
    rover_msgs::msg::MorseStatus _msgRos;
    std::mutex morseMutex;
};

#endif  // MORSE_INPUT_HPP