#include "rover_can_lib/can_bus_manager.hpp"

#if defined(ESP32)

namespace RoverCanLib
{
    CanBusManager::CanBusManager(uint16_t deviceId_,
                                 gpio_num_t txPin_,
                                 gpio_num_t rxPin_,
                                 void (*canMsgCallbackFunc_)(CanBusManager *canBusManager_, const twai_message_t *msgPtr_),
                                 gpio_num_t pinStatusLED_,
                                 twai_mode_t nodeMode_,
                                 twai_timing_config_t configSpeed_)
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

    CanBusManager::~CanBusManager() {}

    void CanBusManager::init()
    {
        ASSERT(twai_start() != ESP_OK);
        // Sending the state of _errorStateMsg which should be OK, this make sure
        // the master resets the node error state after a node reboot
        this->sendMsg(&_errorStateMsg);

        _canBusState = eCanBusStatus::idle;

        if (_withLed)
        {
            _statusLed.init(1, 0.0f);
        }
    }

    void CanBusManager::sendMsg(twai_message_t *msg_)
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

    void CanBusManager::sendMsg(uint32_t id_, uint8_t *data_, uint8_t dataLenght_, bool validateIntegrity_)
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
        COPY_ARRAY(data_, msg.data, dataLenght_);

        this->sendMsg(&msg);
    }

    void CanBusManager::readMsg(OUT twai_message_t *msg)
    {
        twai_message_t message;
        switch (twai_receive(&message, pdMS_TO_TICKS(0u)))
        {
        case ESP_OK:
            // Follow execution
            break;

        case ESP_ERR_TIMEOUT:
            // No new msgs
            *msg = RoverCanLib::Helpers::getErrorIdMsg();
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

    void CanBusManager::update(void)
    {
        if (_withLed)
        {
            this->updateLedStatus();
        }

        this->updateHeartbeat();
        this->updateCallbackMsg();
        this->updateWatchdog();
    }

    void CanBusManager::updateLedStatus(void)
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

        _lastCanBusState = _canBusState;
    }

    void CanBusManager::updateHeartbeat(void)
    {
        if (_timerHeartbeat.isDone())
        {
            this->sendMsg(&_msgHeartbeat);
        }
    }

    void CanBusManager::updateCallbackMsg(void)
    {
        twai_message_t msg;
        for (;;)
        {
            this->readMsg(&msg);

            if (msg.identifier == 0x00)
            {
                // No more msgs
                break;
            }
            else if (msg.identifier == (uint8_t)Constant::eDeviceId::MASTER_COMPUTER_UNIT)
            {
                if (msg.data[(uint8_t)Constant::eDataIndex::MSG_ID] == (uint8_t)Constant::eMsgId::HEARTBEAT)
                {
                    this->resetWatchDog();
                }
            }
            else
            {
                _canMsgCallbackFunc(this, &msg);
            }
        }
    }

    void CanBusManager::updateWatchdog(void)
    {
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

    void CanBusManager::setWarningFlag(void)
    {
        _flagWarning = true;
    }

    void CanBusManager::resetWatchDog(void)
    {
        _canBusState = _canBusState == eCanBusStatus::idle ? eCanBusStatus::running : _canBusState;
        _watchDogAlive = true;
    }

    bool CanBusManager::isOk(void)
    {
        return (_canBusState == running || _canBusState == warning);
    }
}

#endif // !defined(ESP32)
