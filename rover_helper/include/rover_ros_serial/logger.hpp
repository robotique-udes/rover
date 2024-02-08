#ifndef __LOGGER_HPP__
#define __LOGGER_HPP__

#if defined(ESP32)
#include "Arduino.h"
#endif // defined(ESP32)

#include "rover_ros_serial/base_objects.hpp"

namespace RoverRosSerial
{
    class SerialLogger : protected RoverRosSerial::Msg
    {
    private:
        struct sMsgLogger
        {
            uint8_t severity;
            char msg[93];
        };
        union uMsgLogger
        {
            sMsgLogger packetMsg;
            uint8_t packetData[sizeof(sMsgLogger)];
        };

    public:
        SerialLogger()
        {
            uHeader.header.type = Constant::eHeaderType::log;
            uHeader.header.length = sizeof(uMsgLogger::packetData);

            for (uint8_t i = 0; i < sizeof(uMsg.packetMsg.msg); i++)
            {
                uMsg.packetMsg.msg[i] = '\0';
            }
        }
#if defined(ESP32)
        SerialLogger(HardwareSerial *serial_ = &Serial) : Msg(serial_)
        {
            uHeader.header.type = Constant::eHeaderType::log;
            uHeader.header.length = sizeof(uMsgLogger::packetData);

            for (uint8_t i = 0; i < sizeof(uMsg.packetMsg.msg); i++)
            {
                uMsg.packetMsg.msg[i] = '\0';
            }
        }
        ~SerialLogger() {}

        void log(eLoggerLevel severity, const char *str)
        {
            uint8_t sizeOfMsg = static_cast<uint8_t>(strlen(str)) + 1u;

            uMsg.packetMsg.severity = severity;
            memcpy(this->uMsg.packetMsg.msg, str, sizeOfMsg);

            uHeader.header.length = sizeof(severity) + sizeOfMsg;

            this->sendMsg();
        }
#endif // defined(ESP32)
        uMsgLogger uMsg;

    private:
        uint8_t *getSerializedData(void)
        {
            return uMsg.packetData;
        }

        uint8_t getSerializedDataSize(void)
        {
            return uHeader.header.length;
        }
    };

#if defined(ESP32)
    static SerialLogger Logger(&Serial);
#endif
}
#endif // __LOGGER_HPP__
