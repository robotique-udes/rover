#ifndef __LOGGER_H__
#define __LOGGER_H__

#include <rcl/rcl.h>
#include <rcl_interfaces/msg/log.h>
#include "helpers/macros.h"

// Make sure code will execute fine even if LOGGER_LOWEST_LEVEL isn't defined
#ifndef LOGGER_LOWEST_LEVEL
#define LOGGER_LOWEST_LEVEL 0
#endif

// This class creates a publisher and publish msgs to rosout to log msgs the ROS
// terminal running the node
typedef enum eLoggerLevel
{
    DEBUG = rcl_interfaces__msg__Log__DEBUG,
    INFO = rcl_interfaces__msg__Log__INFO,
    WARN = rcl_interfaces__msg__Log__WARN,
    ERROR = rcl_interfaces__msg__Log__ERROR,
    FATAL = rcl_interfaces__msg__Log__FATAL
} eLoggerLevel;

class Logger
{
private:
    rcl_publisher_t _pubLogger;
    bool _alive = false;

    char *_nodeName = NULL;
    char *_ns = NULL;

public:
    Logger() {}
    ~Logger() {}

    bool createLogger(rcl_node_t *node, const char *nodeName_, const char *ns_)
    {
        _nodeName = (char *)(nodeName_);
        _ns = (char *)(ns_);

        RCLC_RET_ON_ERR(rclc_publisher_init_default(&_pubLogger,
                                                    node,
                                                    ROSIDL_GET_MSG_TYPE_SUPPORT(rcl_interfaces, msg, Log),
                                                    NAME_LOG_TOPIC));

        _alive = true;
        return true;
    }

    void destroyLogger(rcl_node_t *node)
    {
        _alive = false;
        REMOVE_WARN_UNUSED(rcl_publisher_fini(&_pubLogger, node));
    }

    void log(eLoggerLevel lvl_, const char *file_, const char *function_, int line_, const char *str_, ...)
    {
        if (lvl_ < LOGGER_LOWEST_LEVEL)
        {
            return;
        }
        rcl_interfaces__msg__Log msg;

        msg.level = lvl_;

        uint8_t buffer_size = 0;
        if (_nodeName == NULL || _ns == NULL)
        {
            buffer_size = sizeof("node_name_not_set");
        }
        else
        {
            buffer_size = strlen(_ns) + 1 + strlen(_nodeName);
        }

        char buffer[buffer_size] = "\0";
        if (_nodeName == NULL || _ns == NULL)
        {
            strncat(buffer, "node_name_not_set", sizeof("node_name_not_set"));
        }
        else
        {
            strncat(buffer, _ns, strlen(_ns) + 1);
            strncat(buffer, "/", sizeof("/"));
            strncat(buffer, _nodeName, strlen(_nodeName) + 1);
        }
        msg.name.data = buffer;

        msg.name.size = strlen(msg.name.data);

        msg.file.data = (char *)file_;
        msg.file.size = strlen(msg.file.data);

        msg.function.data = (char *)function_;
        msg.function.size = strlen(msg.function.data);

        msg.line = line_;

        // Buffer size is arbitrairy now,
        // TODO  might want to evaluate actual necessary

        va_list strArgs;
        va_start(strArgs, str_);
        char msgBuffer[snprintf(NULL, 0, str_, strArgs) + 1];

        vsnprintf(msgBuffer, sizeof(msgBuffer), str_, strArgs);
        msg.msg.data = (char *)msgBuffer;
        msg.msg.size = strlen(msg.msg.data);

        REMOVE_WARN_UNUSED(rcl_publish(&_pubLogger, &msg, NULL));

        va_end(strArgs);
    }

    bool isAlive()
    {
        return _alive;
    }
} Logger;

#endif // __LOGGER_H__
