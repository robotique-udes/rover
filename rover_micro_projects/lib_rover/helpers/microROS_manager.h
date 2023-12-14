#ifndef __MICRO_ROS_MANAGER_H__
#define __MICRO_ROS_MANAGER_H__

#include "helpers/timer.h"

// This class is wrapper that connections with micro-ros-agent to simplify
// microROS developpement. Virtual class for future wrapper implementation with
// perhaps predefined parameters
class MicroROSManager
{
private:
    enum eConnectionStates
    {
        WAITING_AGENT,
        AGENT_AVAILABLE,
        AGENT_CONNECTED,
        AGENT_DISCONNECTED
    };

    eConnectionStates _connectionState = eConnectionStates::WAITING_AGENT;
    TimerMillis _timerCheckReconnect;
    TimerMillis _timerCheckDisconnect;

    virtual bool createEntities(void) = 0;
    virtual void destroyEntities(void) = 0;

public:
    MicroROSManager(uint32_t connectionValidationInterval = 200UL, uint32_t reconnectionInterval = 500UL)
    {
        TimerMillis _timerCheckReconnect = TimerMillis(reconnectionInterval);
        TimerMillis _timerCheckDisconnect = TimerMillis(connectionValidationInterval);
    }
    ~MicroROSManager() {}

    // Tries to spin passed executor, if for some reason it can't, it will try 
    // to reconnect the node to ROS each time this method is called
    void spinSome(rclc_executor_t *executor, uint32_t timeout_mS)
    {
        switch (_connectionState)
        {
        case WAITING_AGENT:
            if (_timerCheckReconnect.done())
            {
                LOG(DEBUG, "Checking if MicroROS agent is available");
                _connectionState = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;
            }
            break;
        case AGENT_AVAILABLE:
            _connectionState = (true == this->createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
            if (_connectionState == WAITING_AGENT)
            {
                LOG(DEBUG, "Agent unavailable, destroying entities...");
                this->destroyEntities();
            };
            break;
        case AGENT_CONNECTED:
            if (_timerCheckDisconnect.done())
            {
                _connectionState = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
            }

            if (_connectionState == AGENT_CONNECTED)
            {
                rclc_executor_spin_some(executor, RCL_MS_TO_NS(timeout_mS));
            }
            break;
        case AGENT_DISCONNECTED:
            LOG(DEBUG, "Agent disconnected, destroying entities...");
            this->destroyEntities();
            _connectionState = WAITING_AGENT;
            break;
        default:
            break;
        }

        if (_connectionState == AGENT_CONNECTED)
        {
            digitalWrite(LED_BUILTIN, 1);
        }
        else
        {
            digitalWrite(LED_BUILTIN, 0);
        }
    }

    int8_t getConnectionState()
    {
        return _connectionState;
    }
};

// This child class of MicroROSManager allow the user to define custom
// createEntities and destroyEntities function that are called when the
// connection state change. This allow creation of custom entities (pub, sub,
// srv, timers, etc.) easily from the "main".
class MicroROSManagerCustom : public MicroROSManager
{
private:
    bool (*_createEntities)(void);
    void (*_destroyEntities)(void);

    bool createEntities(void)
    {
        return (*_createEntities)();
    }
    void destroyEntities(void)
    {
        (*_destroyEntities)();
    }

public:
    MicroROSManagerCustom(bool (*createEntities_)(void),
                          void (*destroyEntities_)(void),
                          uint32_t connectionValidationInterval = 200UL,
                          uint32_t reconnectionInterval = 500UL) : MicroROSManager(connectionValidationInterval, reconnectionInterval)
    {
        if (createEntities_ == NULL || destroyEntities_ == NULL)
        {
            while (true)
            {
                LOG(WARN, "Functions can't be NULL pointers: %i");
                delay(1000);
            }
        }
        _createEntities = createEntities_;
        _destroyEntities = destroyEntities_;

        TimerMillis _timerCheckReconnect = TimerMillis(reconnectionInterval);
        TimerMillis _timerCheckDisconnect = TimerMillis(connectionValidationInterval);
    }

    ~MicroROSManagerCustom() {}
};

#endif // __MICRO_ROS_MANAGER_H__
