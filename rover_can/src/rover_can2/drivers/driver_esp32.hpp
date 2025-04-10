#ifndef DRIVER_ESP32_HPP
#define DRIVER_ESP32_HPP

#include "rover_can2/drivers/driver_base.hpp"

#include "rover_lib2/helpers/circular_buffer.hpp"
#include "rover_lib2/helpers/log.hpp"
#include "rover_lib2/helpers/assert.hpp"

#include "driver/gpio.h"
#include "driver/twai.h"

DEFINE_LOG_NODE(DriverESP32, Logger::eNodeState::ON);

#warning TODO: Add status LED

namespace RoverCan2::Drivers
{
    class DriverESP32 : public DriverBase<DriverESP32>
    {
        static constexpr twai_timing_config_t CAN_SPEED_CONFIG = TWAI_TIMING_CONFIG_1MBITS();
        static constexpr twai_mode_t TWAI_MODE = TWAI_MODE_NORMAL;
        static constexpr twai_filter_config_t TWAI_ID_FILTER = TWAI_FILTER_CONFIG_ACCEPT_ALL();

        static constexpr TickType_t MESSAGE_RECV_TIMEOUT = 0U;  // Doesn't wait if no msg available

        static constexpr size_t CAN_DATA_LENGTH = TWAI_FRAME_MAX_DLC;
        static_assert(Constant::CAN_MAX_DATA_LENGTH == CAN_DATA_LENGTH);

        enum eState : size_t
        {
            UNINSTALLED,
            INSTALLED,
            RUNNING,
        };

      public:
        DriverESP32(gpio_num_t ioRx_, gpio_num_t ioTx_):
            _ioRx(ioRx_),
            _ioTx(ioTx_),
            _state(eState::UNINSTALLED)
        {
        }

        void __init(void)
        {
            this->installDriver();
            this->startDriver();
        }

        void __update(void)
        {
            switch (_state)
            {
                case eState::UNINSTALLED:
                    this->installDriver();
                    [[fallthrough]];
                case eState::INSTALLED:
                    this->startDriver();
                    [[fallthrough]];
                case eState::RUNNING:
                    this->processNewMessage();
                    break;
            }
        }

        size_t _getAvailableMessagesNb(void) const
        {
            return _msgBuffer.size();
        }

        std::optional<CanMsg> _getMsg(void)
        {
            return _msgBuffer.getValue();
        }

        bool _sendMsg(const CanMsg& canMsg_)
        {
            twai_message_t twaiMsg;
            twaiMsg.identifier = static_cast<uint32_t>(canMsg_.getCanID());
            twaiMsg.extd = 0U;
            twaiMsg.rtr = 0U;           // Data frame
            twaiMsg.ss = 0U;            // Not single shot, retry if bus not ready
            twaiMsg.self = 0U;          // Echo mode off
            twaiMsg.dlc_non_comp = 0U;  // Classic frames
            twaiMsg.data_length_code = canMsg_.dataLength + TO_UNDERLYING(RoverCan2::Constant::eDataIndex::START_OF_DATA);

            if (twaiMsg.data_length_code > Constant::CAN_MAX_DATA_LENGTH)
            {
                LOG_ERROR(Logger::Nodes::DriverESP32,
                          "Implementation error, can msg data size (%u) is bigger than max (%u)",
                          twaiMsg.data_length_code,
                          Constant::CAN_MAX_DATA_LENGTH);

                return false;
            }
            else if (twaiMsg.data_length_code < TO_UNDERLYING(RoverCan2::Constant::eDataIndex::START_OF_DATA))
            {
                LOG_ERROR(Logger::Nodes::DriverESP32,
                          "Implementation error, can msg data size (%u) is lower than min (%u)",
                          twaiMsg.data_length_code,
                          TO_UNDERLYING(RoverCan2::Constant::eDataIndex::START_OF_DATA));

                return false;
            }

            std::memcpy(twaiMsg.data, canMsg_.msgData.data(), twaiMsg.data_length_code);
            esp_err_t statusCode = twai_transmit(&twaiMsg, static_cast<TickType_t>(0UL));

            switch (statusCode)
            {
                case ESP_OK:
                    LOG_DEBUG(Logger::Nodes::DriverESP32,
                              "Msg queued for transmission successfully, ID: %u, MsgID: %u, ContentID %u",
                              canMsg_.getCanID(),
                              canMsg_.getMsgID(),
                              canMsg_.getMsgContentID());
                    return true;
                case ESP_ERR_INVALID_ARG:
                    LOG_ERROR(Logger::Nodes::DriverESP32,
                              "Driver queued msg for transmission, invalid arguments. Implementation error");
                    break;
                case ESP_ERR_TIMEOUT:
                    LOG_WARN(Logger::Nodes::DriverESP32, "Couldn't queue msg for transmission, TX buffer full, dropping");
                    break;
                case ESP_FAIL:
                    LOG_WARN(Logger::Nodes::DriverESP32,
                             "Couldn't queue msg for transmission, TX queue is disabled and another message is currently "
                             "transmitting, dropping");
                    break;
                case ESP_ERR_INVALID_STATE:
                    LOG_WARN(Logger::Nodes::DriverESP32,
                             "Couldn't queue msg for transmission, driver has fallen in invalid state, trying to recover...");
                    break;
                case ESP_ERR_NOT_SUPPORTED:
                    LOG_ERROR(Logger::Nodes::DriverESP32,
                              "Couldn't queue msg for transmission, driver is configured in receive only");
                    break;
                default:
                    LOG_ERROR(Logger::Nodes::DriverESP32,
                              "Couldn't queue msg for transmission, unknowned unhandled error %u",
                              statusCode);
                    break;
            }

            return false;
        }

      private:
        void installDriver(void)
        {
            twai_general_config_t genConfig = TWAI_GENERAL_CONFIG_DEFAULT(_ioTx, _ioRx, TWAI_MODE);
            twai_timing_config_t timingConfig = CAN_SPEED_CONFIG;
            twai_filter_config_t IDFilterConfig = TWAI_ID_FILTER;

            esp_err_t successCode = twai_driver_install(&genConfig, &timingConfig, &IDFilterConfig);
            switch (successCode)
            {
                case ESP_OK:
                    LOG_DEBUG(Logger::Nodes::DriverESP32, "Twai driver installed successfully");
                    LOG_DEBUG(
                        Logger::Nodes::DriverESP32,
                        "Using TX pin: %u and RX pin: %u. Friendly reminder, the CAN specs specifies MCU_RX pin into TRANS_RX "
                        "pin and MCU_TX pin into TRANS_TX pin and NOT RX/TX crossover like on uart.",
                        TO_UNDERLYING(_ioTx),
                        TO_UNDERLYING(_ioRx));
                    _state = eState::INSTALLED;
                    break;
                case ESP_ERR_INVALID_STATE:
                    LOG_WARN(Logger::Nodes::DriverESP32,
                             "Can't install twai driver in current state (%u)",
                             TO_UNDERLYING(_state));
                    break;
                case ESP_ERR_INVALID_ARG:
                    ASSERT("Can't install twai driver with specified arguments");
                    break;
                case ESP_ERR_NO_MEM:
                    ASSERT("Can't install twai driver... no more memory");
                    break;
                default:
                    ASSERT("Can't install twai driver... Unknown error: %d", successCode);
                    break;
            }
        }

        void startDriver(void)
        {
            esp_err_t successCode = twai_start();
            switch (successCode)
            {
                case ESP_OK:
                    LOG_DEBUG(Logger::Nodes::DriverESP32, "Twai driver started successfully");
                    _state = eState::RUNNING;
                    break;
                case ESP_ERR_INVALID_STATE:
                    LOG_WARN(Logger::Nodes::DriverESP32, "Can't install twai driver in current state %u", TO_UNDERLYING(_state));
                    break;
                default:
                    ASSERT("Can't start twai driver... Unknown error: %d", successCode);
                    break;
            }
        }

        void processNewMessage(void)
        {
            twai_message_t message;

#warning TODO: Add bus errors handling
            esp_err_t statusCode = twai_receive(&message, MESSAGE_RECV_TIMEOUT);
            switch (statusCode)
            {
                case ESP_OK:
                    LOG_DEBUG(Logger::Nodes::DriverESP32, "New message received from %u", message.identifier);
                    break;
                case ESP_ERR_TIMEOUT:
                    return;
                case ESP_ERR_INVALID_STATE:
                    LOG_ERROR(Logger::Nodes::DriverESP32, "Can Driver has fallen into an invalid state, trying to recover...");
                    _state = eState::UNINSTALLED;
                    return;
                case ESP_ERR_INVALID_ARG:
                    ASSERT("Invalid arguments, implementation mistake");
                    _state = eState::UNINSTALLED;
                    return;
            }

            if (!twaiMsgValid(message))
            {
                return;
            }

            CanMsg msg(message);
            if (msg.getMsgID() == RoverCan2::Constant::eMsgId::INVALID)
            {
                LOG_WARN(Logger::Nodes::DriverESP32, "Received msg with invalid ID: %u dropping", TO_UNDERLYING(msg.getMsgID()));
                return;
            }

            decltype(_msgBuffer)::eErrorCode status = _msgBuffer.addValue(msg);
            switch (status)
            {
                case decltype(_msgBuffer)::eErrorCode::SUCCESS:
                    break;
                case decltype(_msgBuffer)::eErrorCode::SUCCESS_DATA_LOSS:
                    LOG_WARN(Logger::Nodes::DriverESP32, "Msg buffer full, losing data");
                    break;
                case decltype(_msgBuffer)::eErrorCode::ERROR:
                    LOG_WARN(Logger::Nodes::DriverESP32, "Unknown error");
                    break;
            }
        }

        bool twaiMsgValid(const twai_message_t& msg_)
        {
            if (msg_.extd)
            {
                LOG_WARN(Logger::Nodes::DriverESP32, "Received unsupported extended frame, skipping...");
                return false;
            }
            else if (msg_.rtr)
            {
                LOG_DEBUG(Logger::Nodes::DriverESP32, "Received remote frame, skipping...");
                return false;
            }
            else if (msg_.dlc_non_comp)
            {
                LOG_WARN(Logger::Nodes::DriverESP32, "Received too long data frame, skipping...");
                return false;
            }

            return true;
        }

        const gpio_num_t _ioRx;
        const gpio_num_t _ioTx;

        eState _state;
        CircularBuffer<CanMsg, 10UL> _msgBuffer;
    };
}  // namespace RoverCan2::Drivers

#endif  // DRIVER_ESP32_HPP
