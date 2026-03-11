#include "serial_com.hpp"

SerialCom::SerialCom(int fileDesc_, eBaudRate baudRate_, eDataPerPacket char_,
                     tcflag_t cflags_, uint16_t minChar_, uint16_t timeout_)
                     : _fileDesc(fileDesc_),
                       _baudRate(std::to_underlying(baudRate_)),
                       _char(std::to_underlying(char_)),
                       _cflags(cflags_),
                       _minChar(minChar_),
                       _timeout(timeout_),
                       _state(eState::INACTIVE)
{
    if (!this->serialConfig())
    {
        LOG_ERROR(Logger::Nodes::SerialCom, "Unable to configure the serial port");
    }
    else
    {
        this->_state = eState::ACTIVE;
    }
}

SerialCom::~SerialCom()
{
    if (this->_fileDesc >= 0)
    {
        close(this->_fileDesc);
        this->_fileDesc = -1;
        this->_state = eState::INACTIVE;
    }
}

bool SerialCom::serialConfig()
{
    struct termios tty;

    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(this->_fileDesc, &tty) != 0)
    {
        std::string errorMsg = "tcgetattr failed: " + std::string(strerror(errno));
        LOG_ERROR(Logger::Nodes::SerialCom, errorMsg.c_str());
        this->_state = eState::INACTIVE;
        return false;
    }

    cfsetospeed(&tty, this->_baudRate);
    cfsetispeed(&tty, this->_baudRate);

    this->controlFlagsInit(tty);

    tty.c_lflag = 0;  // Disables all flags
    tty.c_iflag = 0;  // Disables all flags
    tty.c_oflag = 0;  // Disables all flags

    tty.c_cc[VMIN] = this->_minChar;
    tty.c_cc[VTIME] = this->_timeout;

    if (tcsetattr(this->_fileDesc, TCSANOW, &tty) != 0)
    {
        std::string errorMsg = "tcsetattr failed: " + std::string(strerror(errno));
        LOG_ERROR(Logger::Nodes::SerialCom, errorMsg.c_str());
        this->_state = eState::INACTIVE;
        return false;
    }

    return true;
}

bool SerialCom::serialWrite(const std::string& cmd_)
{
    if (this->_state != eState::ACTIVE)
    {
        LOG_ERROR(Logger::Nodes::SerialCom, "Cannot write: serial port is inactive");

        return false;
    }

    ssize_t bytesWritten = write(this->_fileDesc, cmd_.c_str(), cmd_.size());

    if (bytesWritten < 0)
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

    if (this->_state != eState::ACTIVE)
    {
        LOG_ERROR(Logger::Nodes::SerialCom, "Cannot read: serial port is inactive");
        return response;
    }

    char buffer[READING_BUFFER];
    ssize_t n = read(this->_fileDesc, buffer, sizeof(buffer));

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

void SerialCom::controlFlagsInit(termios& tty_)
{
    tty_.c_cflag &= ~CSIZE;
    tty_.c_cflag |= (this->_char | this->_cflags);
}

eState SerialCom::getState() const
{
    return this->_state;
}