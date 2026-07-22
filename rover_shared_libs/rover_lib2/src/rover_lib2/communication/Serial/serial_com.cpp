#if defined(__linux__)

#include "serial_com.hpp"

SerialCom::SerialCom(std::string path_,
                     eBaudRate baudRate_,
                     eDataPerPacket char_,
                     tcflag_t cflags_,
                     uint8_t minChar_,
                     uint8_t timeout_):
    _fileDesc(-1),
    _state(eState::INACTIVE),
    _path(path_),
    _baudRate(baudRate_),
    _char(char_),
    _cflags(cflags_),
    _minChar(minChar_),
    _timeout(timeout_)
{
    assert((cflags_ & CSIZE) == 0 && "data size bits belong in eDataPerPacket, not cflags_");
    _fileDesc = open(path_.c_str(), O_RDWR | O_NOCTTY | O_SYNC | O_CLOEXEC);
    if (_fileDesc < 0)
    {
        LOG_ERROR(Logger::Nodes::SerialCom, ("open failed: " + std::string(strerror(errno))).c_str());
        return;
    }

    if (!this->serialConfig())
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

bool SerialCom::serialConfig()
{
    struct termios tty = {};

    if (tcgetattr(_fileDesc, &tty) != 0)
    {
        std::string errorMsg = "tcgetattr failed: " + std::string(strerror(errno));
        LOG_ERROR(Logger::Nodes::SerialCom, errorMsg.c_str());
        _state = eState::INACTIVE;
        return false;
    }

    cfsetospeed(&tty, std::to_underlying(_baudRate));
    cfsetispeed(&tty, std::to_underlying(_baudRate));

    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= (std::to_underlying(_char) | _cflags);

    tty.c_lflag = 0;  // Disables all flags
    tty.c_iflag = 0;  // Disables all flags
    tty.c_oflag = 0;  // Disables all flags

    tty.c_cc[VMIN] = _minChar;
    tty.c_cc[VTIME] = _timeout;

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
        if (std::chrono::steady_clock::now() - _lastLogDC > LOGGER_THROTTLE_MS)
        {
            LOG_ERROR(Logger::Nodes::SerialCom, "Cannot write: serial port is inactive");
            _lastLogDC = std::chrono::steady_clock::now();
        }
        return false;
    }

    ssize_t bytesWritten = write(_fileDesc, cmd_.c_str(), cmd_.size());

    if (bytesWritten < 0)
    {
        std::string errorMsg = "Write failed: " + std::string(strerror(errno));
        LOG_ERROR(Logger::Nodes::SerialCom, errorMsg.c_str());
        return false;
    }
    else if (bytesWritten != static_cast<ssize_t>(cmd_.size()))
    {
        std::string errorMsg = "Write failed: incomplete writing (bytesWritten: " + std::to_string(bytesWritten)
                               + " / cmd_.size(): " + std::to_string(cmd_.size());
        LOG_ERROR(Logger::Nodes::SerialCom, errorMsg.c_str());
        return false;
    }

    return true;
}

std::optional<std::string> SerialCom::serialRead()
{
    if (_state != eState::ACTIVE)
    {
        if (std::chrono::steady_clock::now() - _lastLogDC > LOGGER_THROTTLE_MS)
        {
            LOG_ERROR(Logger::Nodes::SerialCom, "Cannot read: serial port is inactive");
        }
        return std::nullopt;
    }

    const std::chrono::time_point<std::chrono::steady_clock> deadline
        = std::chrono::steady_clock::now() + std::chrono::milliseconds(READ_TIMEOUT_MS);

    while (true)
    {
        const size_t pos = _rxBuffer.find(FRAME_TERMINATOR);
        if (pos != std::string::npos)
        {
            std::string frame = _rxBuffer.substr(0, pos);
            _rxBuffer.erase(0, pos + 1);
            return frame;
        }

        if (std::chrono::steady_clock::now() > deadline)
        {
            LOG_ERROR(Logger::Nodes::SerialCom, "Readin request timed out waiting for full frame");
            return std::nullopt;
        }

        char buffer[READING_BUFFER];
        ssize_t n = read(_fileDesc, buffer, sizeof(buffer));

        if (n < 0)
        {
            std::string errorMsg = "Read failed: " + std::string(strerror(errno));
            LOG_ERROR(Logger::Nodes::SerialCom, errorMsg.c_str());
            return std::nullopt;
        }
        else if (n == 0)
        {
            continue;
        }
        else
        {
            _rxBuffer.append(buffer, n);
            if (_rxBuffer.size() > MAX_RX_SIZE)
            {
                LOG_ERROR(Logger::Nodes::SerialCom,
                          "Reading request never received frame terminator. Disregarding all previous data");
                _rxBuffer.clear();
                return std::nullopt;
            }
        }
    }
}

bool SerialCom::reconnect()
{
    if (_fileDesc >= 0)
    {
        close(_fileDesc);
        _fileDesc = -1;
    }
    _state = eState::INACTIVE;

    _fileDesc = open(_path.c_str(), O_RDWR | O_NOCTTY | O_SYNC | O_CLOEXEC);
    if (_fileDesc < 0)
    {
        if (std::chrono::steady_clock::now() - _lastLogDC > LOGGER_THROTTLE_MS)
        {
            LOG_ERROR(Logger::Nodes::SerialCom, ("open failed: " + std::string(strerror(errno))).c_str());
        }
        return false;
    }

    if (!serialConfig())
    {
        LOG_ERROR(Logger::Nodes::SerialCom, "Unable to configure the serial port");
        close(_fileDesc);
        _fileDesc = -1;
        return false;
    }

    if (tcflush(_fileDesc, TCIOFLUSH) == -1)
    {
        LOG_WARN(Logger::Nodes::SerialCom, ("tcflush failed: " + std::string(strerror(errno))).c_str());
    }

    _state = eState::ACTIVE;
    _rxBuffer.clear();
    return true;
}

void SerialCom::flushInput()
{
    _rxBuffer.clear();
    tcflush(_fileDesc, TCIFLUSH);
}

eState SerialCom::getState() const
{
    return _state;
}

#endif