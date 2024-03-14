#ifndef __CAN_BUS_MANAGER_HPP__
#define __CAN_BUS_MANAGER_HPP__

#if defined(ESP32)

#include "rover_can_lib/msgs/msg.hpp"
#include "rover_can_lib/msgs/error_state.hpp"
#include "rover_can_lib/msgs/heartbeat.hpp"
#include "rover_can_lib/config.hpp"

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
                      gpio_num_t pinStatusLED_ = GPIO_NUM_NC,
                      twai_mode_t nodeMode_ = twai_mode_t::TWAI_MODE_NORMAL,
                      twai_timing_config_t configSpeed_ = TWAI_TIMING_CONFIG_1MBITS());
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
        void sendMsg(RoverCanLib::Msgs::Msg *msg, bool validateIntegrity_ = false)
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
        /// this will allow us to see which node is having probleme problems
        void sendErrorCode(RoverCanLib::Constant::eInternalErrorCode errCode)
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

    private:
        void updateLedStatus(void);
        void updateHeartbeat(void);
        void updateCallbackMsg(void);
        void updateWatchdog(void);

        uint16_t _id;
        bool _withLed = false;
        LedBlinker _statusLed;
        eCanBusStatus _lastCanBusState;
        eCanBusStatus _canBusState = eCanBusStatus::notInit;
        bool _flagWarning = false;
        Timer<unsigned long, millis> _timerHeartbeat;
        RoverCanLib::Msgs::Heartbeat _msgHeartbeat;
        Timer<unsigned long, millis> _timerWatchdog = Timer<unsigned long, millis>(RoverCanLib::Constant::WATCHDOG_TIMEOUT_MS);
        bool _watchDogAlive = false;
        void (*_canMsgCallbackFunc)(CanBusManager *canBusManager_, const twai_message_t *msgPtr_) = NULL;
        RoverCanLib::Msgs::ErrorState _errorStateMsg;
    };
}

#endif // !defined(ESP32)
#endif // __CAN_BUS_MANAGER_HPP__
