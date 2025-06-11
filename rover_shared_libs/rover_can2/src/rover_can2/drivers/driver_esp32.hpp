#ifndef DRIVER_ESP32_HPP
#define DRIVER_ESP32_HPP

#include "rover_can2/drivers/driver_base.hpp"

#include "rover_lib2/helpers/circular_buffer.hpp"
#include "rover_lib2/helpers/log.hpp"
#include "rover_lib2/helpers/assert.hpp"
#include "rover_lib2/LED/led_blinker.hpp"
#include "rover_lib2/helpers/watchdog.hpp"

#include "driver/gpio.h"
#include "driver/twai.h"

DEFINE_LOG_NODE(DriverESP32, Logger::eNodeState::OFF);

namespace RoverCan2::Drivers
{
    template<typename LedBlinkerT_>
    class DriverESP32 : public DriverBase<DriverESP32<LedBlinkerT_>>
    {
        VALIDATE_BASE_TYPE(LED::LedBlinkerT, LedBlinkerT_);

        static constexpr twai_timing_config_t CAN_SPEED_CONFIG = TWAI_TIMING_CONFIG_1MBITS();
        static constexpr twai_mode_t TWAI_MODE = TWAI_MODE_NORMAL;
        static constexpr twai_filter_config_t TWAI_ID_FILTER = TWAI_FILTER_CONFIG_ACCEPT_ALL();

        static constexpr TickType_t MESSAGE_RECV_TIMEOUT = 0U;  // Doesn't wait if no msg available
        static constexpr uint16_t DEFAULT_TX_QUEUE_LENGTH = 5UL;

        static constexpr size_t CAN_DATA_LENGTH = TWAI_FRAME_MAX_DLC;
        static_assert(Constant::CAN_MAX_DATA_LENGTH == CAN_DATA_LENGTH);

        static constexpr uint64_t RECV_WATCHDOG_TIMEOUT_MS
            = 2ULL * 1'000ULL / static_cast<uint64_t>(Constant::MASTER_HEARTBEAT_RATE_HZ);

        enum eState : size_t
        {
            UNINSTALLED,
            BUS_OFF,
            RUNNING,
            TX_QUEUE_FULL,
            INVALID_STATE,
        };

      public:
        DriverESP32(gpio_num_t ioRx_,
                    gpio_num_t ioTx_,
                    LedBlinkerT_* led_ = nullptr,
                    uint16_t txQueueLength_ = DEFAULT_TX_QUEUE_LENGTH):
            _ioRx(ioRx_),
            _ioTx(ioTx_),
            _txQueueLength(static_cast<uint32_t>(txQueueLength_)),
            _state(eState::UNINSTALLED),
            _led(led_),
            _recvWatchdog(RECV_WATCHDOG_TIMEOUT_MS)
        {
        }

        void __init(void)
        {
            this->installDriver();
            this->startDriver();

            if (_led)
            {
                _led->setPattern(RoverCan2::Constant::LedPatterns::DRIVER_NOT_STARTED);
                _led->init();
            }
        }

        void __update(void)
        {
            if (_led)
            {
                _led->update();
                this->updateStatusLed();
            }

            switch (_state)
            {
                case eState::UNINSTALLED:
                    this->installDriver();
                    [[fallthrough]];
                case eState::BUS_OFF:
                    this->startDriver();
                    [[fallthrough]];
                case eState::RUNNING:
                    [[fallthrough]];
                case eState::TX_QUEUE_FULL:
                    this->processNewMessage();
                    break;

                case eState::INVALID_STATE:
                    this->handleRecovery();
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
            if (_state < eState::RUNNING)
            {
                LOG_WARN(Logger::Nodes::DriverESP32,
                         "Can't send msg, driver is not in a valid state to send messages. Expected state >= %u but current "
                         "state is: %u. Msg dropped",
                         TO_UNDERLYING(eState::RUNNING),
                         TO_UNDERLYING(_state));
                return false;
            }

            twai_message_t twaiMsg;
            twaiMsg.identifier = static_cast<uint32_t>(canMsg_.getCanID());
            twaiMsg.extd = 0U;
            twaiMsg.rtr = 0U;           // Data frame
            twaiMsg.ss = 0U;            // Not single shot, retry if bus not ready
            twaiMsg.self = 0U;          // Echo mode off
            twaiMsg.dlc_non_comp = 0U;  // Classic frames
            twaiMsg.data_length_code = canMsg_.dataLength;

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
                    _state = eState::TX_QUEUE_FULL;
                    LOG_WARN(Logger::Nodes::DriverESP32, "Couldn't queue msg for transmission, TX buffer full, dropping");
                    break;
                case ESP_FAIL:
                    LOG_WARN(Logger::Nodes::DriverESP32,
                             "Couldn't queue msg for transmission, TX queue is disabled and another message is currently "
                             "transmitting, dropping");
                    break;
                case ESP_ERR_INVALID_STATE:
                    _state = eState::INVALID_STATE;
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
        void handleRecovery(void)
        {
            if (_state == eState::INVALID_STATE)
            {
                twai_status_info_t currentState;
                esp_err_t successCode = twai_get_status_info(&currentState);
                switch (successCode)
                {
                    case ESP_OK:
                        switch (currentState.state)
                        {
                            case twai_state_t::TWAI_STATE_STOPPED:
                                LOG_INFO(Logger::Nodes::DriverESP32, "Bus recovered successfully! Driver will restart");
                                _state = eState::BUS_OFF;
                                break;

                            case twai_state_t::TWAI_STATE_RUNNING:
                                LOG_INFO(Logger::Nodes::DriverESP32, "Bus recovered successfully! Driver is running");
                                _state = eState::RUNNING;
                                break;

                            case twai_state_t::TWAI_STATE_BUS_OFF:
                                LOG_WARN(Logger::Nodes::DriverESP32, "Bus has fallen in invalid state, initiation recovery...");
                                if (twai_initiate_recovery() == ESP_OK)
                                {
                                    _state = eState::INVALID_STATE;
                                }
                                else
                                {
                                    _state = eState::UNINSTALLED;
                                }
                                break;

                            case twai_state_t::TWAI_STATE_RECOVERING:
                                LOG_DEBUG(Logger::Nodes::DriverESP32, "Recovery in progress...");
                                break;
                        }
                        break;

                    case ESP_ERR_INVALID_ARG:
                        ASSERT_MSG(
                            "In recovery handling, can't get current twai state with specified arguments. Implementation error");
                        break;
                    case ESP_ERR_INVALID_STATE:
                        LOG_WARN(Logger::Nodes::DriverESP32,
                                 "Can't get TWAI status info, driver is not installed. Implementation error");
                        _state = eState::UNINSTALLED;
                        break;
                }
            }
        }

        void updateStatusLed(void)
        {
            if (_led)
            {
                switch (_state)
                {
                    case eState::UNINSTALLED:
                        [[fallthrough]];
                    case eState::BUS_OFF:
                        _led->setPattern(RoverCan2::Constant::LedPatterns::DRIVER_NOT_STARTED);
                        break;
                    case eState::INVALID_STATE:
                        _led->setPattern(RoverCan2::Constant::LedPatterns::DRIVER_INTERNAL_ERROR);
                        break;
                    case eState::RUNNING:
                        if (_recvWatchdog.isOk())
                        {
                            _led->setPattern(RoverCan2::Constant::LedPatterns::RUNNING_OK);
                        }
                        else
                        {
                            _led->setPattern(RoverCan2::Constant::LedPatterns::WATCHDOG_TRIGGER);
                        }
                        break;
                    case eState::TX_QUEUE_FULL:
                        _led->setPattern(RoverCan2::Constant::LedPatterns::TX_QUEUE_FULL);
                        break;
                }
            }
        }

        void installDriver(void)
        {
            twai_general_config_t genConfig = TWAI_GENERAL_CONFIG_DEFAULT(_ioTx, _ioRx, TWAI_MODE);
            genConfig.tx_queue_len = _txQueueLength;
            twai_timing_config_t timingConfig = CAN_SPEED_CONFIG;
            twai_filter_config_t IDFilterConfig = TWAI_ID_FILTER;

            esp_err_t successCode = twai_driver_install(&genConfig, &timingConfig, &IDFilterConfig);
            switch (successCode)
            {
                case ESP_OK:
                    LOG_DEBUG(Logger::Nodes::DriverESP32, "Twai driver installed successfully");
                    LOG_DEBUG(Logger::Nodes::DriverESP32,
                              "Using TX pin: %i and RX pin: %i. Friendly reminder, the CAN specs specifies MCU_RX<-TRANS_RX "
                              "and MCU_TX->TRANS_TX and NOT RX->TX|TX->RX crossover like on UART.",
                              TO_UNDERLYING(_ioTx),
                              TO_UNDERLYING(_ioRx));
                    _state = eState::BUS_OFF;
                    break;
                case ESP_ERR_INVALID_STATE:
                    LOG_WARN(Logger::Nodes::DriverESP32,
                             "Can't install twai driver in current state (%u)",
                             TO_UNDERLYING(_state));
                    break;
                case ESP_ERR_INVALID_ARG:
                    ASSERT_MSG("Can't install twai driver with specified arguments");
                    _state = eState::UNINSTALLED;
                    break;
                case ESP_ERR_NO_MEM:
                    ASSERT_MSG("Can't install twai driver... no more memory");
                    _state = eState::UNINSTALLED;
                    break;
                default:
                    ASSERT_MSG_ARGS("Can't install twai driver... Unknown error: %d", successCode);
                    _state = eState::UNINSTALLED;
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
                    ASSERT_MSG_ARGS("Can't start twai driver... Unknown error: %d", successCode);
                    break;
            }
        }

        void processNewMessage(void)
        {
            twai_message_t message;

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
                    _state = eState::INVALID_STATE;
                    return;
                case ESP_ERR_INVALID_ARG:
                    ASSERT_MSG("Invalid arguments, implementation error");
                    _state = eState::UNINSTALLED;
                    return;
            }

            if (!twaiMsgValid(message))
            {
                return;
            }
            _recvWatchdog.reset();  // Watchdog should still be valid even if msgs are not

            CanMsg msg(message);
            if (msg.getMsgID() == RoverCan2::Constant::eMsgId::INVALID)
            {
                LOG_WARN(Logger::Nodes::DriverESP32, "Received msg with invalid ID: %u dropping", TO_UNDERLYING(msg.getMsgID()));
                return;
            }

            typename decltype(_msgBuffer)::eErrorCode status = _msgBuffer.addValue(msg);
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
        const uint32_t _txQueueLength;

        eState _state;
        LedBlinkerT_* const _led;

        CircularBuffer<CanMsg, 10UL> _msgBuffer;
        Watchdog<uint64_t, Time::millis> _recvWatchdog;
    };

    template<typename LedBlinkerT_>
    DriverESP32(gpio_num_t ioRx_, gpio_num_t ioTx_, LedBlinkerT_* led_ = nullptr) -> DriverESP32<LedBlinkerT_>;
}  // namespace RoverCan2::Drivers

#endif  // DRIVER_ESP32_HPP
