#include "morse_input.hpp"
#include "rover_can2/msgs/morse_code.hpp"
#include <rover_lib2/helpers/constants.hpp>

MorseInput::MorseInput(RoverCan2::Constant::eDeviceId IdCan_):
    DerivedT(IdCan_,
             RoverCan2::Publisher<RoverCan2::Msgs::MorseCode>(),
             RoverCan2::SubscriberMember<RoverCan2::Msgs::MorseStatus, MorseInput>(*this, &MorseInput::CB_CAN_MorseStatus))
{
}

void MorseInput::rosElementInit()
{
    _sub_morseCode = this->getAttachedNode()->create_subscription<rover_msgs::msg::MorseCode>(
        TOPIC_MORSE_CODE,
        QOS_DEFAULT,
        [this](const rover_msgs::msg::MorseCode& rosMsg_)
        {
            this->CB_ROS_morseCode(rosMsg_);
        });

    _pub_morseStatus = this->getAttachedNode()->create_publisher<rover_msgs::msg::MorseStatus>(TOPIC_MORSE_STATUS, QOS_DEFAULT);

    _timer_statusPublisher = this->getAttachedNode()->create_wall_timer(std::chrono::milliseconds(STATUS_PUBLISH_PERIOD_MS),
                                                                        [this]()
                                                                        {
                                                                            if (_pub_morseStatus)
                                                                            {
                                                                                _pub_morseStatus->publish(_msgRos);
                                                                            }
                                                                        });
}

void MorseInput::rosElementClean()
{
    if (_sub_morseCode)
    {
        _sub_morseCode.reset();
    }

    if (_pub_morseStatus)
    {
        _pub_morseStatus.reset();
    }

    if (_timer_statusPublisher)
    {
        _timer_statusPublisher.reset();
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
    for(int i = 0; i < 30; i++){
    this->sendMsg(_nextMorseInputMsg);}    
}

void MorseInput::CB_CAN_MorseStatus(const RoverCan2::Msgs::MorseStatus& msgCan_)
{
    _msgRos.is_busy = msgCan_.getData().is_busy;
}
