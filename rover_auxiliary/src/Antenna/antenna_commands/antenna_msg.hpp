#ifndef ANTENNA_MSG_HPP
#define ANTENNA_MSG_HPP

struct sSignalInfos
{
    bool connected = false;
    float rssi = 0.0f;
    float rxRate = 0.0f;
    float txRate = 0.0f;
};

#endif  // ANTENNA_MSG_HPP