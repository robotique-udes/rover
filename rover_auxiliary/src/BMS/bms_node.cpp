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
    SerialCom terminal;

    _cellVolt.clear();
    _cellVolt.reserve(MAX_CELL);

    fileDesc = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_SYNC);

    if (fileDesc < 0)
    {
        std::cout << "Error encountered when opening the serial" << std::endl;
    }

    tcflush(fileDesc, TCIOFLUSH);

    terminal.serialWrite("?A\r");
    ampSerialOutput = terminal.serialRead();
    terminal.serialWrite("?V\r");
    cellsVoltSerialOutput = serialRead();

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


