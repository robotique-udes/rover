#ifndef SERIAL_COM_H
#define SERIAL_COM_H

#include "rclcpp/rclcpp.hpp"
#include "rover_lib2/helpers/constants.hpp"
#include <fstream>
#include <utility>
#include <iostream>
#include <vector>
#include <fcntl.h>
#include <termios.h>

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

enum class eCharSize : tcflag_t
{
    C_S5 = CS5,
    C_S6 = CS6,
    C_S7 = CS7,
    C_S8 = CS8
};

class SerialCom
{
  public:
    SerialCom(eBaudRate baudRate_,
              eCharSize char_,
              bool twoStopBit_,
              bool enRead_,
              bool ignModem_,
              bool parity_,
              bool oddParity_,
              uint16_t minChar_,
              uint16_t timeout_);
    void serialConfig(int fileDesc_);
    void serialWrite(int fileDesc_, const std::string& cmd_);
    std::string serialRead(int fileDesc_);
    void controlFlagsInit(termios& tty_);

    speed_t _baudRate;
    tcflag_t _char;
    bool _twoStopBit;
    bool _enRead;
    bool _ignModem;
    bool _parity;
    bool _oddParity;
    uint16_t _minChar;
    uint16_t _timeout;
};

#endif