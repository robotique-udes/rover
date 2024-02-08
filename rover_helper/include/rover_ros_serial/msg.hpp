#ifndef __MSG_HPP__
#define __MSG_HPP__

#include "rover_ros_serial/rover_ros_serial.hpp"

namespace RoverRosSerial
{
    class Msg
    {
    public:
        Msg() {}
#if defined(ESP32)
        Msg(HardwareSerial *serial_ = &Serial)
        {
            if (serial_ == NULL)
            {
                pSerial = &Serial;
            }

            pSerial = serial_;
        }

        virtual void sendMsg()
        {
            pSerial->write(Constant::BEGIN);
            pSerial->write(getSerializedHeader(), getSerializedheaderSize());

            // Only send msg if it's not empty
            uint8_t msgLength = getSerializedDataSize();
            if (msgLength > 0)
            {
                pSerial->write(getSerializedData(), getSerializedDataSize());
            }
        }
#endif // defined(ESP32)

    protected:
        Constant::uHeader uHeader;

        virtual uint8_t *getSerializedHeader(void)
        {
            return uHeader.data;
        }

        virtual uint8_t getSerializedheaderSize(void)
        {
            return sizeof(uHeader.data);
        }

        virtual uint8_t *getSerializedData(void) = 0;
        virtual uint8_t getSerializedDataSize(void) = 0;

    private:
#if defined(ESP32)
        HardwareSerial *pSerial = NULL;
#endif // defined(ESP32)
    };
}
#endif // __MSG_HPP__
