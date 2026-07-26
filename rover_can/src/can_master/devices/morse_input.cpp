#include "morse_input.hpp"
#include "rover_can2/msgs/morse_code.hpp"
#include <rover_lib2/helpers/constants.hpp>

MorseInput::MorseInput(RoverCan2::Constant::eDeviceId IdCan_):
    DerivedT(IdCan_, RoverCan2::Publisher<RoverCan2::Msgs::MorseCode>())
{
}

void MorseInput::rosElementInit()
{
    _sub_morseCode = this->getAttachedNode()->create_subscription<rover_msgs::msg::MorseCode>(
        TOPIC_MORSE_CODE,
        QOS_CAMERA,
        [this](const rover_msgs::msg::MorseCode& rosMsg_)
        {
            this->CB_ROS_morseCode(rosMsg_);
        });
}

void MorseInput::rosElementClean()
{
    if (_sub_morseCode)
    {
        _sub_morseCode.reset();
    }
}

std::vector<RoverCan2::Constant::eDeviceId> MorseInput::getManagedDevicesIds(void)
{
    return std::vector{this->getCanId()};
}

void MorseInput::CB_ROS_morseCode(const rover_msgs::msg::MorseCode& rosMsg_)
{
    this->_nextMorseInputMsg.data().start = rosMsg_.start;
    this->_nextMorseInputMsg.data().index = rosMsg_.index;
    this->_nextMorseInputMsg.data().msg_length = rosMsg_.length;
    this->_nextMorseInputMsg.data().character = rosMsg_.character;
    this->_nextMorseInputMsg.data().checksum = rosMsg_.checksum;
    this->sendMsg(_nextMorseInputMsg);
}