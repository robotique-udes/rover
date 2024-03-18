#ifndef __CAN_DEVICE_HPP__
#define __CAN_DEVICE_HPP__

#include "rover_can_lib/config.hpp"
#include "rovus_lib/timer.hpp"
#include "rover_can_lib/msgs/error_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/can_device_status.hpp"

class CanDevice
{
public:
    CanDevice(uint16_t id_, void (*callback_)(uint16_t id_, const can_frame *frameMsg_))
    {
        _id = id_;

        assert(callback_ != NULL);
        _callback = callback_;
    }
    ~CanDevice() {}

    void parseMsg(can_frame *frameMsg)
    {
        switch (frameMsg->data[(uint8_t)RoverCanLib::Constant::eDataIndex::MSG_ID])
        {
        case (uint8_t)RoverCanLib::Constant::eMsgId::HEARTBEAT:
            this->resetWatchdog();
            break;

        case (uint8_t)RoverCanLib::Constant::eMsgId::ERROR_STATE:
            this->setErrorState(frameMsg);
            break;

        default:
            this->_callback(this->getId(), frameMsg);
        }
    }

    uint16_t getId()
    {
        return _id;
    }

private:
    uint16_t _id;
    // rclcpp::Publisher<rover_msgs::msg::CanDeviceStatus>::SharedPtr _pub_CanBusState;
    rover_msgs::msg::CanDeviceStatus _msg_canStatus;
    Chrono<uint64_t, millis> _timerWatchdog;
    void (*_callback)(uint16_t id_, const can_frame *frameMsg_);

    void resetWatchdog()
    {
        if (_timerWatchdog.getTime() > RoverCanLib::Constant::WATCHDOG_TIMEOUT_MS)
        {
            _msg_canStatus.watchdog_ok = false;
        }
        else
        {
            _msg_canStatus.watchdog_ok = true;
        }
        _timerWatchdog.restart();
    }
    void setErrorState(can_frame *frameMsg)
    {
        RoverCanLib::Msgs::ErrorState msg;
        if (msg.parseMsg(frameMsg, this->getLogger()) != RoverCanLib::Constant::eInternalErrorCode::OK)
        {
            RCLCPP_ERROR(this->getLogger(), "Error parsing ErrorCode message, dropping");
        }
        _msg_canStatus.error_state = msg.data.warning ? rover_msgs::msg::CanDeviceStatus::STATUS_WARNING : rover_msgs::msg::CanDeviceStatus::STATUS_OK;
        _msg_canStatus.error_state = msg.data.error ? rover_msgs::msg::CanDeviceStatus::STATUS_ERROR : _msg_canStatus.error_state;
    }
    rclcpp::Logger getLogger()
    {
        std::stringstream ss;
        ss << std::hex << _id; // Set the stream to output in hexadecimal
        return rclcpp::get_logger("0x" + ss.str());
    }
};


#endif // __CAN_DEVICE_HPP__
