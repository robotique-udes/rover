#include "bms_node.hpp"

int main(int argc_, char** argv_)
{
    rclcpp::init(argc_, argv_);
    auto node = std::make_shared<BMSDataNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

BMSDataNode::BMSDataNode():
    Node("bms_info")
{
    _publisher = this->create_publisher<rover_msgs::msg::BmsData>(TOPIC_BMS_DATA, QOS_DEFAULT);

    _timer_publisher = this->create_wall_timer(std::chrono::milliseconds(DELAY_PUBLISHER_MS),
                                               [this](void)
                                               {
                                                   this->callbackBMSData();
                                               });
}

void BMSDataNode::callbackBMSData(void)
{
    rover_msgs::msg::BmsData msg;

    getData();

    msg.battery_amps = _batteryAmps;

    msg.cell_volt = _cellVolt;

    _publisher->publish(msg);
}

void BMSDataNode::getData(void)
{
    std::string ampSerialOutput;
    std::string cellsVoltSerialOutput;
    int fileDesc;
    uint16_t ampIndex = 0;

    _cellVolt.clear();
    _cellVolt.reserve(MAX_CELL);

    fileDesc = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_SYNC);

    if (fileDesc < 0)
    {
        std::cout << "Error encountered when opening the serial" << std::endl;
    }

    serialConfig(fileDesc);
    tcflush(fileDesc, TCIOFLUSH);

    serialWrite(fileDesc, "?A\r");
    ampSerialOutput = serialRead(fileDesc);
    serialWrite(fileDesc, "?V\r");
    cellsVoltSerialOutput = serialRead(fileDesc);

    ampSerialOutput = ampSerialOutput.substr(AMP_START_INDEX);
    cellsVoltSerialOutput = cellsVoltSerialOutput.substr(CELL_START_INDEX);

    while (ampSerialOutput[ampIndex] != ':')
    {
        ampIndex++;
    }

    _batteryAmps = std::stod(ampSerialOutput.substr(0, ampIndex));

    for (uint16_t index = 0; index < MAX_CELL; index++)
    {
        uint16_t cellIndex = 0;

        while (cellsVoltSerialOutput[cellIndex] != ':')
        {
            cellIndex++;
        }
        _cellVolt.push_back(std::stoi(cellsVoltSerialOutput.substr(0, cellIndex)));
        cellsVoltSerialOutput = cellsVoltSerialOutput.substr(cellIndex + 1);
    }

    close(fileDesc);
}

void BMSDataNode::serialConfig(int fileDesc_)
{
    struct termios tty;

    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fileDesc_, &tty) != 0)
    {
        std::cout << "tcgetattr failed" << std::endl;
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;  // Sets 8 bits characters
    tty.c_cflag |= CLOCAL | CREAD;               // Enable receiver and ignore modem control lines
    tty.c_cflag &= ~(PARENB | PARODD);           // Disable parity
    tty.c_cflag &= ~CSTOPB;                      // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;                     // Disables RTS/CTS hardware flow control

    tty.c_lflag = 0;  // Disables all flags
    tty.c_iflag = 0;  // Disables all flags
    tty.c_oflag = 0;  // Disables all flags

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 10;

    if (tcsetattr(fileDesc_, TCSANOW, &tty) != 0)
    {
        std::cout << "tcsetattr failed" << std::endl;
    }
}

void BMSDataNode::serialWrite(int fileDesc_, const std::string& cmd_)
{
    ssize_t bytesWritten = write(fileDesc_, cmd_.c_str(), cmd_.size());
    (void)bytesWritten;
}

std::string BMSDataNode::serialRead(int fileDesc_)
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
