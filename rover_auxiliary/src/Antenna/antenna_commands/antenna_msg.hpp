#ifndef ANTENNA_MSG_HPP
#define ANTENNA_MSG_HPP

struct sAntennaMsg
{
    bool connected = false;
    float rssi = 0.0f;
    float rxRate = 0.0f;
    float txRate = 0.0f;

    void reset(void)
    {
        this->connected = false;
        this->rssi = 0.0f;
        this->rxRate = 0.0f;
        this->txRate = 0.0f;
    }
};

#endif // ANTENNA_MSG_HPP