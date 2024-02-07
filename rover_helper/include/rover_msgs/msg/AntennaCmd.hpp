#ifndef __ANTENNA_CMD_HPP__
#define __ANTENNA_CMD_HPP__

#include "rover_ros_serial/rover_ros_serial.hpp"

class AntennaCmd : protected RoverRosSerial::Msg
{
private:
    struct sAntennaCmd
    {
        bool status;
        float speed;
    };

    union uAntennaCmd
    {
        sAntennaCmd packetMsg;
        uint8_t packetData[sizeof(sAntennaCmd)];
    };

public:
    uAntennaCmd uMsg;

    AntennaCmd()
    {
        uHeader.header.type = RoverRosSerial::Constant::msg + 1;
        uHeader.header.length = sizeof(uAntennaCmd);
    }
    ~AntennaCmd(){}

    uint8_t *getSerializedData(void)
    {
        return uMsg.packetData;
    }

    uint8_t getSerializedDataSize(void)
    {
        return sizeof(uMsg.packetData);
    }
};

#endif __ANTENNA_CMD_HPP__
