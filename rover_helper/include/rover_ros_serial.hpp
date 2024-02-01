#ifndef __ROVER_ROS_SERIAL_HPP__
#define __ROVER_ROS_SERIAL_HPP__

#include <stdint.h>
#include <cstring>

#define ROVER_ROS_SERIAL

namespace RoverRosSerial
{
    namespace Constant
    {
        constexpr uint8_t BEGIN = 128u;

        enum eHeaderType : uint8_t
        {
            notInitialised = 0,
            log = 130u,
            msg = 140u,
            srv = 150u
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

    class MsgLogger : protected RoverRosSerial::Msg
    {
    public:
        struct sMsgLogger
        {

            char msg[93];
        };

        union uMsgLogger
        {
            sMsgLogger packetMsg;
            uint8_t packetData[sizeof(sMsgLogger)];
        };

    public:
        MsgLogger()
        {
            uHeader.header.type = Constant::eHeaderType::log;
            uHeader.header.length = sizeof(uMsgLogger::packetData);

            for (uint8_t i = 0; i < sizeof(uData.packetMsg.msg); i++)
            {
                uData.packetMsg.msg[i] = '\0';
            }
        }
        ~MsgLogger() {}

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
            return uData.packetData;
        }

        uint8_t getSerializedDataSize(void)
        {
            return uHeader.header.length;
        }

        void setLog(const char *str, const uint16_t size)
        {
            memcpy(this->uData.packetMsg.msg, str, size);
        }

        // void sendMsg(HardwareSerial *serial)
        // {
        //     serial->write(Constant::BEGIN);
        //     serial->write(getSerializedHeader(), getSerializedheaderSize());
        //     serial->write(getSerializedData(), getSerializedDataSize());
        // }

        Constant::uHeader uHeader;
        uMsgLogger uData;
    };
}
#endif // __ROVER_ROS_SERIAL_HPP__
