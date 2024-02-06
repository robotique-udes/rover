#ifndef __ROVER_ROS_SERIAL_HPP__
#define __ROVER_ROS_SERIAL_HPP__

#include <stdint.h>
#include <cstring>

namespace RoverRosSerial
{
    namespace Constant
    {
        constexpr uint8_t BEGIN = 128u;

        enum eHeaderType : uint8_t
        {
            notInitialised = 0,
            heartbeat = 129u,
            log = 130u,
            msg = 150u,
            srv = 200u
        };

        struct sHeader
        {
            uint16_t type;
            uint16_t length;
        };

        union uHeader
        {
            sHeader header;
            uint8_t data[sizeof(sHeader)];
        };
    }

    class Msg
    {
    public:
        virtual uint8_t *getSerializedData(void) = 0;
        virtual uint8_t getSerializedDataSize(void) = 0;
    };

    class SerialLogger : protected RoverRosSerial::Msg
    {
    public:
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
        ~SerialLogger() {}

        uint8_t *getSerializedHeader(void)
        {
            return uHeader.data;
        }

        uint8_t getSerializedheaderSize(void)
        {
            return sizeof(uHeader.data);
        }

        uint8_t *getSerializedData(void)
        {
            return uMsg.packetData;
        }

        uint8_t getSerializedDataSize(void)
        {
            return uHeader.header.length;
        }

        void setLog(const char *str, const uint16_t size)
        {
            memcpy(this->uMsg.packetMsg.msg, str, size);
        }

        // void sendMsg(HardwareSerial *serial)
        // {
        //     serial->write(Constant::BEGIN);
        //     serial->write(getSerializedHeader(), getSerializedheaderSize());
        //     serial->write(getSerializedData(), getSerializedDataSize());
        // }

        Constant::uHeader uHeader;
        uMsgLogger uMsg;
    };
}
#endif // __ROVER_ROS_SERIAL_HPP__
