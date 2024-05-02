#ifndef __CAN_BUS_MANAGER_HPP__
#define __CAN_BUS_MANAGER_HPP__

#if defined(ESP32)

#include "rover_can_lib/msgs/msg.hpp"
#include "rover_can_lib/msgs/error_state.hpp"
#include "rover_can_lib/msgs/heartbeat.hpp"
#include "rover_can_lib/config.hpp"

#include "rover_helpers/led_blinker.hpp"

namespace RoverCanLib
{
    /// @brief CanBusManager class: wrapper to help communicate on the canbus
    /// network easily
    class CanBusManager
    {
    public:
        /// @brief Internal status of the can node in relation to it's network
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

        /// @brief Constructor
        /// @param deviceId_ Used for sending internal node msgs with the correct ID
        /// @param txPin_ txGpio pin. Ex for pin D0: GPIO_NUM_0 or
        /// static_cast<gpio_num_t>(0u)
        /// @param rxPin_ rxGpio pin. Ex for pin D0: GPIO_NUM_0 or
        /// static_cast<gpio_num_t>(0u)
        /// @param canMsgCallbackFunc_ Function called each time when a new
        /// message is received, similar to ROS callbacks
        /// @param pinStatusLED_ GPIO on which the led status feedback is
        /// attached Ex for pin D0: GPIO_NUM_0 or static_cast<gpio_num_t>(0u)
        /// @param nodeMode_ twai_mode_t::TWAI_MODE_NORMAL or
        /// twai_mode_t::TWAI_MODE_LISTEN_ONLY
        /// @param configSpeed_ can bus speed 1MB/s recommended
        CanBusManager(uint16_t deviceId_,
                      gpio_num_t txPin_,
                      gpio_num_t rxPin_,
                      void (*canMsgCallbackFunc_)(CanBusManager *canBusManager_, const twai_message_t *msgPtr_),
                      bool disableMasterWatchdog_,
                      gpio_num_t pinStatusLED_ = GPIO_NUM_NC,
                      twai_mode_t nodeMode_ = twai_mode_t::TWAI_MODE_NORMAL,
                      twai_timing_config_t configSpeed_ = TWAI_TIMING_CONFIG_1MBITS());

        /// @brief Destructor
        ~CanBusManager();

        /// @brief Required to call after the constructor
        void init();

        /// @brief Send a already initialised twai_message_t (can frame)
        /// @param msg_ pointer to already initialised twai_message_t
        void sendMsg(twai_message_t *msg_);

        /// @brief Send a message with parameters for automatic initialisation
        /// of the twai_message_t (can frame)
        /// @param id_ id on which the message will be sent
        /// @param data_ pointer to uin8_t array (max 8 bytes)
        /// @param dataLenght_ length of data_
        /// @param validateIntegrity_ Will automatically retry the message
        /// transmission until some node AKC the msg
        void sendMsg(uint32_t id, uint8_t *data_, uint8_t dataLenght_, bool validateIntegrity_ = false);

        /// @brief Send all required can msgs to send a whole RoverCanLib::Msgs
        /// ::Msg
        /// @param msg Pointer to the message RoverCanLib::Msgs::Msg child object
        void sendMsg(RoverCanLib::Msgs::Msg *msg, bool validateIntegrity_ = false);

        /// @brief Check if some message are available and return them if there
        /// are or return an empty message with ID = 0x00 if not
        /// @param msg
        void readMsg(OUT twai_message_t *msg);

        /// @brief Internal calculation, Should be called at each code loop
        void update(void);

        /// @brief Send the canbus manager into warning mode, useful for
        /// debugging. Once it's latch (called once) it won't be unlatch until
        /// next reboot
        void setWarningFlag(void);

        /// @brief Reset the watchdog timeout between two messages
        void resetWatchDog(void);

        /// @brief Return true if the canbus is not in error state, in warning
        /// will return true
        bool isOk(void);

        /// @brief Set an error code and send it to the master from this node ID,
        /// this will allow us to see which node is having problems
        void sendErrorCode(RoverCanLib::Constant::eInternalErrorCode errCode);

    private:
        void updateLedStatus(void);
        void updateHeartbeat(void);
        void updateCallbackMsg(void);
        void updateWatchdog(void);

        uint16_t _id;

        bool _flagWarning = false;
        eCanBusStatus _lastCanBusState;
        eCanBusStatus _canBusState = eCanBusStatus::notInit;

        RoverCanLib::Msgs::Heartbeat _msgHeartbeat;
        RoverHelpers::Timer<unsigned long, millis> _timerHeartbeat;

        bool _watchDogAlive = false;
        bool _disableMasterWatchdog = false;
        RoverHelpers::Timer<unsigned long, millis> _timerWatchdog = RoverHelpers::Timer<unsigned long, millis>(RoverCanLib::Constant::WATCHDOG_TIMEOUT_MS);

        void (*_canMsgCallbackFunc)(CanBusManager *canBusManager_, const twai_message_t *msgPtr_) = NULL;

        RoverCanLib::Msgs::ErrorState _errorStateMsg;

        bool _withLed = false;
        LedBlinker _statusLed;
    };

    CanBusManager::CanBusManager(uint16_t deviceId_,
                                 gpio_num_t txPin_,
                                 gpio_num_t rxPin_,
                                 void (*canMsgCallbackFunc_)(CanBusManager *canBusManager_, const twai_message_t *msgPtr_),
                                 bool disableMasterWatchdog_,
                                 gpio_num_t pinStatusLED_,
                                 twai_mode_t nodeMode_,
                                 twai_timing_config_t configSpeed_)
        : _statusLed(pinStatusLED_),
          _timerHeartbeat(static_cast<unsigned long>(1000.0f / static_cast<float>(RoverCanLib::Constant::HEARTBEAT_FREQ)))
    {
        _id = deviceId_;

        twai_general_config_t configGen = TWAI_GENERAL_CONFIG_DEFAULT(txPin_, rxPin_, nodeMode_);
        // Smaller queue often caused problems.
        configGen.rx_queue_len = 20;
        configGen.tx_queue_len = 20;
        // Filter are not supported at the stage
        twai_filter_config_t configFilter = TWAI_FILTER_CONFIG_ACCEPT_ALL();

        ASSERT(twai_driver_install(&configGen, &configSpeed_, &configFilter) != ESP_OK, "Error installing CanBus Driver");

        ASSERT(canMsgCallbackFunc_ == NULL);
        _canMsgCallbackFunc = canMsgCallbackFunc_;

        if (pinStatusLED_ != GPIO_NUM_NC)
        {
            _withLed = true;
        }

        _disableMasterWatchdog = disableMasterWatchdog_;
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
        // Sending msg with very samll timeout, as a guideline we don't execute
        // blocking call with motors as it can lead to security issues. But 1ms
        // maximum block should be safe.
        switch (twai_transmit(msg_, pdMS_TO_TICKS(0)))
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

    void CanBusManager::sendMsg(RoverCanLib::Msgs::Msg *msg, bool validateIntegrity_)
    {
        twai_message_t canMsg;
        canMsg.identifier = _id;
        canMsg.ss = validateIntegrity_;
        canMsg.extd = 0;
        canMsg.dlc_non_comp = 0;
        canMsg.self = 0;
        canMsg.rtr = 0;

        // Start at 1 (after NOT_USED)
        for (uint8_t i = 1; i < msg->getMsgIDNb(); i++)
        {
            this->sendErrorCode(msg->getMsg(i, &canMsg));
            this->sendMsg(&canMsg);
        }
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

            if (msg.identifier == 0x00) // No more msgs
            {
                break;
            }
            else if (msg.identifier == (uint8_t)Constant::eDeviceId::MASTER_COMPUTER_UNIT) // Global msg coming from Master
            {
                if (!_disableMasterWatchdog)
                {
                    if (msg.data[(uint8_t)Constant::eDataIndex::MSG_ID] == (uint8_t)Constant::eMsgId::HEARTBEAT)
                    {
                        this->resetWatchDog();
                    }
                }

                // When receiving ErrorState from master, each node must send back their current status, this will then
                // be forwarded to a ros topic by the master. But only answer on a last element of a message to not
                // flood the network for no reasons
                if (msg.data[(uint8_t)Constant::eDataIndex::MSG_ID] == (uint8_t)Constant::eMsgId::ERROR_STATE &&
                    RoverCanLib::Helpers::msgContentIsLastElement<Msgs::ErrorState>(&msg))
                {
                    LOG(INFO, "Asked by master to send error code, sending...");
                    Constant::eInternalErrorCode errorState;
                    if (_canBusState > eCanBusStatus::ERROR_DELIMITER)
                    {
                        this->sendErrorCode(Constant::eInternalErrorCode::ERROR);
                    }
                    else if (_canBusState == eCanBusStatus::warning)
                    {
                        this->sendErrorCode(Constant::eInternalErrorCode::WARNING);
                    }
                    else
                    {
                        this->sendErrorCode(Constant::eInternalErrorCode::OK);
                    }
                    LOG(INFO, "Done sending");
                }
            }
            else // Msgs for this node with custom execution
            {   
                if (msg.data_length_code < (uint8_t)RoverCanLib::Constant::eDataIndex::START_OF_DATA + 1u)
                {
                    LOG(WARN, "Ill formed msg, dropping...");
                    this->sendErrorCode(RoverCanLib::Constant::eInternalErrorCode::WARNING);
                    continue;
                }
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

    void CanBusManager::sendErrorCode(RoverCanLib::Constant::eInternalErrorCode errCode)
    {
        if (errCode == Constant::eInternalErrorCode::WARNING)
        {
            this->setWarningFlag();
            _errorStateMsg.data.warning = true;
            this->sendMsg(&_errorStateMsg, false);
        }

        if (errCode == Constant::eInternalErrorCode::ERROR)
        {
            _errorStateMsg.data.error = true;
            this->sendMsg(&_errorStateMsg, false);
        }
    }
}

#endif // !defined(ESP32)
#endif // __CAN_BUS_MANAGER_HPP__
