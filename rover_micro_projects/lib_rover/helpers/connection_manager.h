#ifndef __CONNECTION_MANAGER_H__
#define __CONNECTION_MANAGER_H__

#include <Arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <micro_ros_platformio.h>
#include <rmw_microros/rmw_microros.h>

#include "helpers/macros.h"
#include "helpers/timer.h"

class ConnectionManager
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

    // Must return true if no error and false if any step as had an error
    bool (*createEntities)(void);
    void (*destroyEntities)(void);

public:
    ConnectionManager(bool (*createEntities_)(void), void (*destroyEntities_)(void), uint32_t connectionValidationInterval = 200UL , uint32_t reconnectionInterval = 500UL)
    {
        if (createEntities_ == NULL || destroyEntities_ == NULL)
        {
            while(true)
            {
                int a = 0;
                WARN("Functions can't be NULL pointers: %i", a);
                delay(1000);
            }
        }
        createEntities = createEntities_;
        destroyEntities = destroyEntities_;
        TimerMillis _timerCheckReconnect = TimerMillis(reconnectionInterval);
        TimerMillis _timerCheckDisconnect = TimerMillis(connectionValidationInterval);
    }
    ~ConnectionManager() {}

    void spinSome(rclc_executor_t *executor, uint32_t timeout_mS)
    {
        if (this->createEntities != NULL && this->destroyEntities != NULL)
        {
            switch (_connectionState)
            {
            case WAITING_AGENT:
                if (_timerCheckReconnect.done())
                {
                    _connectionState = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;
                }
                break;
            case AGENT_AVAILABLE:
                _connectionState = (true == (*createEntities)()) ? AGENT_CONNECTED : WAITING_AGENT;
                if (_connectionState == WAITING_AGENT)
                {
                    (*destroyEntities)();
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
                (*destroyEntities)();
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
    }
};

// DO NOT USE
// Do a while loop or, even better, a for(EVER) loop in your setup() function
// This will keep all setup variables inside the scope and will remove the need
// to declare them globally
void loop() {}

#endif // __CONNECTION_MANAGER_H__
