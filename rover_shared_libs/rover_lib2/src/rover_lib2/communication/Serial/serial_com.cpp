#include "serial_com.hpp"

SerialCom::SerialCom(eBaudRate baudRate_,
                     eCharSize char_,
                     bool twoStopBit_,
                     bool enRead_,
                     bool ignModem_,
                     bool parity_,
                     bool oddParity_,
                     uint16_t minChar_,
                     uint16_t timeout_)
{
    _baudRate = static_cast<speed_t>(baudRate_);
    _char = static_cast<tcflag_t>(char_);
    _twoStopBit = twoStopBit_;
    _enRead = enRead_;
    _ignModem = ignModem_;
    _parity = parity_;
    _oddParity = oddParity_;
    _minChar = minChar_;
    _timeout = timeout_;
}

void SerialCom::serialConfig(int fileDesc_)
{
    struct termios tty;

    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fileDesc_, &tty) != 0)
    {
        std::cout << "tcgetattr failed" << std::endl;
    }

    cfsetospeed(&tty, _baudRate);
    cfsetispeed(&tty, _baudRate);

    controlFlagsInit(tty);

    tty.c_lflag = 0;  // Disables all flags
    tty.c_iflag = 0;  // Disables all flags
    tty.c_oflag = 0;  // Disables all flags

    tty.c_cc[VMIN] = _minChar;
    tty.c_cc[VTIME] = _timeout;

    if (tcsetattr(fileDesc_, TCSANOW, &tty) != 0)
    {
        std::cout << "tcsetattr failed" << std::endl;
    }
}

void SerialCom::serialWrite(int fileDesc_, const std::string& cmd_)
{
    ssize_t bytesWritten = write(fileDesc_, cmd_.c_str(), cmd_.size());
    (void)bytesWritten;
}

std::string SerialCom::serialRead(int fileDesc_)
{
    char buffer[256];
    std::string response;

    ssize_t n = read(fileDesc_, buffer, sizeof(buffer));

    if (n > 0)
    {
        response.assign(buffer, n);
    }

    return response;
}

void SerialCom::controlFlagsInit(termios& tty_)
{
    tty_.c_cflag &= ~CSIZE;
    tty_.c_cflag |= _char;

    if (_twoStopBit)
    {
        tty_.c_cflag |= CSTOPB;
    }
    if (_enRead)
    {
        tty_.c_cflag |= CREAD;
    }
    if (_ignModem)
    {
        tty_.c_cflag |= CLOCAL;
    }
    if (_parity)
    {
        tty_.c_cflag |= PARENB;

        if (_oddParity)
        {
            tty_.c_cflag |= PARODD;
        }
    }
}