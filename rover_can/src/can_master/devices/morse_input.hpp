#ifndef MORSE_INPUT_HPP
#define MORSE_INPUT_HPP

#include "can_master/master_device.hpp"

#include <rover_can2/rover_can2.hpp>
#include <rover_can2/msgs/morse_input.hpp>
#include <rover_msgs/msg/morse_code.hpp>

class MorseInput : public RoverCan2::Device<RoverCan2::Publisher<RoverCan2::Msgs::MorseInput>>,
               public MasterDevice
{
    static constexpr const char* TOPIC_MORSE_CODE = "/base/gui/morse_code";

  public:
    MorseInput(RoverCan2::Constant::eDeviceId IdCan_);

  private:
    void rosElementInit() override;
    void rosElementClean() override;
    std::vector<RoverCan2::Constant::eDeviceId> getManagedDevicesIds(void) override;

    void CB_ROS_morseCode(const rover_msgs::msg::MorseCode& rosMsg_);

    const uint8_t _idCamCameraControlMsg;

    RoverCan2::Msgs::MorseInput _nextMorseInputMsg;


    rclcpp::Subscription<rover_msgs::msg::MorseCode>::SharedPtr _sub_morseCode;
};

#endif  // MORSE_INPUT_HPP