#ifndef SERIAL_COM_H
#define SERIAL_COM_H

#include "rclcpp/rclcpp.hpp"
#include "rover_lib2/helpers/constants.hpp"
#include "rover_lib2/helpers/log.hpp"
#include <fstream>
#include <utility>
#include <iostream>
#include <vector>
#include <fcntl.h>
#include <termios.h>
#include <cstring>
#include <cerrno>
#include <unistd.h>

enum class eBaudRate : speed_t
{
    B_57600 = B57600,
    B_115200 = B115200,
    B_230400 = B230400,
    B_460800 = B460800,
    B_500000 = B500000,
    B_576000 = B576000,
    B_921600 = B921600,
    B_1000000 = B1000000,
    B_1152000 = B1152000,
    B_1500000 = B1500000,
    B_2000000 = B2000000,
    B_2500000 = B2500000,
    B_3000000 = B3000000,
    B_3500000 = B3500000,
    B_4000000 = B4000000
};

enum class eDataPerPacket : tcflag_t
{
    FIVE_BITS = CS5,
    SIX_BITS = CS6,
    SEVEN_BITS = CS7,
    EIGHT_BITS = CS8
};

enum class eState : size_t
{
    INACTIVE,
    ACTIVE
};

DEFINE_LOG_NODE(SerialCom, Logger::eNodeState::ON);

class SerialCom
{
  public:
    SerialCom(int fileDesc_,
              eBaudRate baudRate_= eBaudRate::B_1152000,
              eDataPerPacket char_ = eDataPerPacket::EIGHT_BITS,
              tcflag_t cflags_ = CREAD | CLOCAL,
              uint16_t minChar_ = 0,
              uint16_t timeout_ = 10);
    ~SerialCom();
    bool serialWrite(const std::string& cmd_);
    std::string serialRead();
    eState getState() const;

 private:
    static const uint16_t READING_BUFFER = 256;
    bool serialConfig();
    void controlFlagsInit(termios& tty_);
    int _fileDesc;
    speed_t _baudRate;
    tcflag_t _char;
    tcflag_t _cflags;
    uint16_t _minChar;
    uint16_t _timeout;
    eState _state;
};

#endif