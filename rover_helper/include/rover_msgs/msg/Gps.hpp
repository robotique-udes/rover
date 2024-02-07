#ifndef __GPS_HPP__
#define __GPS_HPP__

#include "rover_ros_serial/rover_ros_serial.hpp"

namespace RoverRosSerial::rover_msgs::msg
{
    class Gps : public RoverRosSerial::Msg
    {
    private:
        struct sGps
        {
            // From GPS
            float latitude;
            float longitude;
            float height;
            float heading_gps;
            float heading_track;
            float speed;
            uint8_t satellite;

            // From magnetometer
            float heading;
        };

        union uGps
        {
            sGps packetMsg;
            uint8_t packetData[sizeof(sGps)];
        };

    public:
        uGps uMsg;

        Gps()
        {
            uHeader.header.type = RoverRosSerial::Constant::msg + 1;
            uHeader.header.length = sizeof(uGps);
        }
        ~Gps() {}

        uint8_t *getSerializedData(void)
        {
            return uMsg.packetData;
        }

        uint8_t getSerializedDataSize(void)
        {
            return sizeof(uMsg.packetData);
        }
    };
}
#endif // __ANTENNA_CMD_HPP__
