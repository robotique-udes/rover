#ifndef __CAN_BUS_MANAGER_HPP__
#define __CAN_BUS_MANAGER_HPP__

#if defined(ESP32)

#include "driver/twai.h"

#include "helpers/helpers.hpp"
#include "rover_can_lib/rover_can_lib.hpp"

namespace RoverCanLib
{
    class CanBusManager
    {
    public:
        enum eCanBusStatus : uint8_t
        {
            notInit,
            idle,
            running,
            warning,
            ERROR_DELIMITER,
            watchdogError,
            error
        };

        CanBusManager(uint16_t deviceId_,
                      gpio_num_t txPin_,
                      gpio_num_t rxPin_,
                      void (*canMsgCallbackFunc_)(CanBusManager *canBusManager_, const twai_message_t *msgPtr_),
                      gpio_num_t pinStatusLED_ = GPIO_NUM_NC,
                      twai_mode_t nodeMode_ = twai_mode_t::TWAI_MODE_NORMAL,
                      twai_timing_config_t configSpeed_ = TWAI_TIMING_CONFIG_1MBITS())
            : _statusLed(pinStatusLED_),
              _timerHeartbeat(static_cast<unsigned long>(1000.0f / static_cast<float>(RoverCanLib::Constant::HEARTBEAT_FREQ)))
        {
            _id = deviceId_;

            twai_general_config_t configGen = TWAI_GENERAL_CONFIG_DEFAULT(txPin_, rxPin_, nodeMode_);
            // Filter are not supported at the stage
            twai_filter_config_t configFilter = TWAI_FILTER_CONFIG_ACCEPT_ALL();

            ASSERT(twai_driver_install(&configGen, &configSpeed_, &configFilter) != ESP_OK, "Error installing CanBus Driver");

            ASSERT(canMsgCallbackFunc_ == NULL);
            _canMsgCallbackFunc = canMsgCallbackFunc_;

            if (pinStatusLED_ != GPIO_NUM_NC)
            {
                _withLed = true;
            }
        }
        ~CanBusManager() {}
        void init()
        {
            ASSERT(twai_start() != ESP_OK);

            _canBusState = eCanBusStatus::idle;

            if (_withLed)
            {
                _statusLed.init(1, 0.0f);
            }
        }
        void sendMsg(twai_message_t *msg_)
        {
            // Sending msg with timeout to zero, as a guideline we don't execute
            // blocking call with motors as it can lead to security issues.
            switch (twai_transmit(msg_, 0u))
            {
            case ESP_OK:
                // Follow execution
                break;

            case ESP_ERR_TIMEOUT:
                _canBusState = eCanBusStatus::error;
                LOG(ERROR, "TX queue is full!");
                return;

            case ESP_FAIL:
                LOG(ERROR, "Race condition! esp is already sending a msg and the queue is disable");
                _canBusState = eCanBusStatus::error;
                return;

            case ESP_ERR_NOT_SUPPORTED:
                // See twai_general_config_t in constructor to change mode
                LOG(WARN, "CanBus manager is configured in read only mode, can't send message");
                return;

            case ESP_ERR_INVALID_ARG:
                _canBusState = eCanBusStatus::error;
                ASSERT(true, "Invalid arguments in function call");
                break;

            case ESP_ERR_INVALID_STATE:
                _canBusState = eCanBusStatus::error;
                ASSERT(true, "Canbus/Twai controller has fallen in an invalid state");
                break;

            default:
                _canBusState = eCanBusStatus::error;
                ASSERT(true, "Unknowned error, shouldn't ever fall here");
            }
        }
        void sendMsg(uint32_t id_, uint8_t *data_, uint8_t dataLenght_, bool validateIntegrity_ = false)
        {
            if (dataLenght_ > 8)
            {
                LOG(ERROR, "Can't send CAN-FD frame yet, not all our adapter supports it");
                return;
            }

            twai_message_t msg;
            msg.extd = 0;
            msg.dlc_non_comp = 0;
            msg.self = 0;
            msg.ss = validateIntegrity_;
            msg.rtr = 0;

            msg.identifier = id_;
            msg.data_length_code = dataLenght_;
            this->copyArray(data_, msg.data, dataLenght_);

            this->sendMsg(&msg);
        }
        void readMsg(OUT twai_message_t *msg)
        {
            twai_message_t message;
            switch (twai_receive(&message, pdMS_TO_TICKS(0u)))
            {
            case ESP_OK:
                // Follow execution
                break;

            case ESP_ERR_TIMEOUT:
                // No new msgs
                *msg = this->getErrorIdMsg();
                return;

            case ESP_ERR_INVALID_ARG:
                _canBusState = eCanBusStatus::error;
                ASSERT(true, "Invalid arguments in function call");
                break;

            case ESP_ERR_INVALID_STATE:
                _canBusState = eCanBusStatus::error;
                ASSERT(true, "Canbus/Twai controller has fallen in an invalid state");
                break;

            default:
                _canBusState = eCanBusStatus::error;
                ASSERT(true, "Unknowned error, shoudln't ever fall here");
            }

            // Process received message
            if (message.extd)
            {
                LOG(WARN, "Extended can ID frame are not supported by our adapter, dropping message");
            }

            *msg = message;
        }
        void update(void)
        {
            // LOG(INFO, "%u", _canBusState);
            // Update status led
            if (_withLed)
            {
                if (_flagWarning && _canBusState < ERROR_DELIMITER)
                {
                    _canBusState = eCanBusStatus::warning;
                }

                if (_canBusState != _lastCanBusState)
                {
                    switch (_canBusState)
                    {
                    case notInit:
                        _statusLed.setOff();
                        break;

                    case idle:
                        _statusLed.setOn();
                        break;

                    case running:
                        _statusLed.setBlink(1, 50.0f);
                        break;

                    case warning:
                        _statusLed.setBlink(2, 50.0f);
                        break;

                    case watchdogError:
                        _statusLed.setBlink(20, 25.0f);
                        break;

                    case error:
                        _statusLed.setBlink(10, 50.0f);
                        break;

                    default:
                        break;
                    }
                }
            }
            _lastCanBusState = _canBusState;

            // Sends heartbeat
            if (_timerHeartbeat.isDone())
            {
                uint8_t data[] = {0};
                this->sendMsg(_id, data, sizeof(data), false);
            }

            // Read new messages
            twai_message_t msg;
            for (;;)
            {
                readMsg(&msg);

                if (msg.identifier == 0x00)
                {
                    // No more msgs
                    break;
                }
                else
                {
                    _canMsgCallbackFunc(this, &msg);
                }
            }

            if (_timerWatchdog.isDone())
            {
                if (_canBusState != eCanBusStatus::notInit && _canBusState != eCanBusStatus::idle && _watchDogAlive)
                {
                    if (_canBusState == eCanBusStatus::watchdogError)
                    {
                        _canBusState = _flagWarning ? eCanBusStatus::warning : eCanBusStatus::running;
                    }
                }
                else if (!_watchDogAlive)
                {
                    _canBusState = eCanBusStatus::watchdogError;
                }

                _watchDogAlive = false;
            }
        }
        void setWarningFlag(void)
        {
            _flagWarning = true;
        }
        void resetWatchDog(void)
        {
            _canBusState = _canBusState == eCanBusStatus::idle ? eCanBusStatus::running : _canBusState;
            _watchDogAlive = true;
        }
        bool isOk(void)
        {
            return (_canBusState == running || _canBusState == warning);
        }

    private:
        // Copy size_ bytes from src_ array data into dest_ array, it's the users
        // job to make sure the size of src_ and dest_ are valid (longer than size_)
        void copyArray(uint8_t *src_, uint8_t *dest_, uint8_t size_)
        {
            for (uint8_t i = 0; i < size_; i++)
            {
                dest_[i] = src_[i];
            }
        }
        // Return a msg with id == 0 which means it shouldn't be considered, we than
        // don't have to initialize a whole empty message
        twai_message_t getErrorIdMsg(void)
        {
            twai_message_t msg;
            msg.identifier = 0u;

            return msg;
        }

        uint16_t _id;
        LedBlinker _statusLed;
        bool _withLed = false;
        eCanBusStatus _lastCanBusState;
        eCanBusStatus _canBusState = eCanBusStatus::notInit;
        bool _flagWarning = false;

        Timer<unsigned long, millis> _timerHeartbeat;
        Timer<unsigned long, millis> _timerWatchdog = Timer<unsigned long, millis>(RoverCanLib::Constant::WATCHDOG_TIMEOUT_MS);
        bool _watchDogAlive = false;
        void (*_canMsgCallbackFunc)(CanBusManager *canBusManager_, const twai_message_t *msgPtr_) = NULL;
    };
}

#endif // !defined(ESP32)
#endif // __CAN_BUS_MANAGER_HPP__
