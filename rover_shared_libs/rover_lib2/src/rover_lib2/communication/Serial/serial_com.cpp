#include "serial_com.hpp"

SerialCom::SerialCom(const char* path_,
                     eBaudRate baudRate_,
                     eDataPerPacket char_,
                     tcflag_t cflags_,
                     uint16_t minChar_,
                     uint16_t timeout_):
    _fileDesc(-1),
    _state(eState::INACTIVE)
{
    _fileDesc = open(path_, O_RDWR | O_NOCTTY | O_SYNC | O_CLOEXEC);
    if (_fileDesc < 0)
    {
        LOG_ERROR(Logger::Nodes::SerialCom, ("open failed: " + std::string(strerror(errno))).c_str());
        return;
    }

    if (!this->serialConfig(baudRate_, char_, cflags_, minChar_, timeout_))
    {
        LOG_ERROR(Logger::Nodes::SerialCom, "Unable to configure the serial port");
        close(_fileDesc);
        _fileDesc = -1;
        return;
    }

    if (tcflush(_fileDesc, TCIOFLUSH) == -1)
    {
        std::string errorMsg = "tcflush failed: " + std::string(strerror(errno));
        LOG_WARN(Logger::Nodes::SerialCom, errorMsg.c_str());
    }

    _state = eState::ACTIVE;
}

SerialCom::~SerialCom()
{
    if (_fileDesc >= 0)
    {
        close(_fileDesc);
    }
}

bool SerialCom::serialConfig(eBaudRate baudRate_, eDataPerPacket char_, tcflag_t cflags_, uint16_t minChar_, uint16_t timeout_)
{
    struct termios tty;

    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(_fileDesc, &tty) != 0)
    {
        std::string errorMsg = "tcgetattr failed: " + std::string(strerror(errno));
        LOG_ERROR(Logger::Nodes::SerialCom, errorMsg.c_str());
        _state = eState::INACTIVE;
        return false;
    }

    cfsetospeed(&tty, std::to_underlying(baudRate_));
    cfsetispeed(&tty, std::to_underlying(baudRate_));

    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= (std::to_underlying(char_) | cflags_);

    tty.c_lflag = 0;  // Disables all flags
    tty.c_iflag = 0;  // Disables all flags
    tty.c_oflag = 0;  // Disables all flags

    tty.c_cc[VMIN] = minChar_;
    tty.c_cc[VTIME] = timeout_;

    if (tcsetattr(_fileDesc, TCSANOW, &tty) != 0)
    {
        std::string errorMsg = "tcsetattr failed: " + std::string(strerror(errno));
        LOG_ERROR(Logger::Nodes::SerialCom, errorMsg.c_str());
        _state = eState::INACTIVE;
        return false;
    }

    return true;
}

bool SerialCom::serialWrite(const std::string& cmd_)
{
    if (_state != eState::ACTIVE)
    {
        LOG_ERROR(Logger::Nodes::SerialCom, "Cannot write: serial port is inactive");

        return false;
    }

    ssize_t bytesWritten = write(_fileDesc, cmd_.c_str(), cmd_.size());

    if (bytesWritten < 0 || bytesWritten != static_cast<ssize_t>(cmd_.size()))
    {
        std::string errorMsg = "Write failed: " + std::string(strerror(errno));
        LOG_ERROR(Logger::Nodes::SerialCom, errorMsg.c_str());
        return false;
    }

    return true;
}

std::string SerialCom::serialRead()
{
    std::string response;

    if (_state != eState::ACTIVE)
    {
        LOG_ERROR(Logger::Nodes::SerialCom, "Cannot read: serial port is inactive");
        return response;
    }

    char buffer[READING_BUFFER];
    ssize_t n = read(_fileDesc, buffer, sizeof(buffer));

    if (n > 0)
    {
        response.assign(buffer, n);
    }
    else if (n < 0)
    {
        std::string errorMsg = "Read failed: " + std::string(strerror(errno));
        LOG_ERROR(Logger::Nodes::SerialCom, errorMsg.c_str());
    }

    return response;
}

eState SerialCom::getState() const
{
    return _state;
}