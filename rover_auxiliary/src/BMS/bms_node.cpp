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
    this->_publisher = create_publisher<rover_msgs::msg::BmsData>(TOPIC_BMS_DATA, QOS_DEFAULT);

    this->_timer_publisher = create_wall_timer(std::chrono::milliseconds(DELAY_PUBLISHER_MS),
                                               [this](void)
                                               {
                                                   this->callbackBMSData();
                                               });
}

void BMSDataNode::callbackBMSData(void)
{
    rover_msgs::msg::BmsData msg;

    this->getData();

    msg.battery_amps = this->_batteryAmps;

    msg.cell_volt = this->_cellVolt;

    this->_publisher->publish(msg);
}

void BMSDataNode::getData(void)
{
    std::string ampSerialOutput;
    std::string voltSerialOutput;
    int fileDesc;
    uint16_t ampIndex = 0;

    this->_cellVolt.clear();
    this->_cellVolt.reserve(MAX_CELL);

    fileDesc = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_SYNC);

    if (tcflush(fileDesc, TCIOFLUSH) == -1)
    {
        std::string errorMsg = "tcflush failed: " + std::string(strerror(errno));
        RCLCPP_WARN(this->get_logger(), errorMsg.c_str());
    }

    SerialCom terminal(fileDesc);

    terminal.serialWrite("?A\r");
    ampSerialOutput = terminal.serialRead();
    terminal.serialWrite("?V\r");
    voltSerialOutput = terminal.serialRead();

    for (size_t i = 0; i < ampSerialOutput.size(); i++)
    {
        std::cout << ampSerialOutput[i];
    }

    std::cout << std::endl;

    for (size_t i = 0; i < voltSerialOutput.size(); i++)
    {
        std::cout << voltSerialOutput[i];
    }

    std::cout << std::endl;

    while (ampSerialOutput[ampIndex] != ':')
    {
        ampIndex++;
    }

    this->_batteryAmps = std::stod(ampSerialOutput.substr(0, ampIndex));

    for (uint16_t index = 0; index < MAX_CELL; index++)
    {
        uint16_t cellIndex = 0;

        while (voltSerialOutput[cellIndex] != ':')
        {
            cellIndex++;
        }
        this->_cellVolt.push_back(std::stoi(voltSerialOutput.substr(0, cellIndex)));
        voltSerialOutput = voltSerialOutput.substr(cellIndex + 1);
    }

    if (close(fileDesc) == -1)
    {
        std::string errorMsg = "unable to close the fileDesc:" + std::string(strerror(errno));
        RCLCPP_WARN(this->get_logger(), errorMsg.c_str());
    }
}

void BMSDataNode::parse(std::string rawOutput, uint16_t dataNumber, std::vector<uint16_t>& dataArray)
{
    uint16_t indexGarb = 0;

    while(indexGarb < rawOutput.size() && rawOutput[indexGarb] != '=')
    {
        indexGarb++;
    }

    rawOutput = rawOutput.substr(indexGarb + 1);

    for (uint16_t i = 0; i < dataNumber; i++)
    {
        uint16_t index = 0;

        while (index < rawOutput.size() && rawOutput[index] != ':')
        {
            index++;
        }
        dataArray.push_back(std::stoi(rawOutput.substr(0, index)));
        rawOutput = rawOutput.substr(index + 1);
    }
}


