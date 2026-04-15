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

    msg.battery_amps = this->_ampArray[AmpIndexType::BATTERY_AMPS];

    std::vector<uint16_t> cellVolts;

    for (uint16_t i = VoltIndexType::CELL_VOLT_START; i < VoltIndexType::CELL_VOLT_END; i++)
    {
        cellVolts.push_back(this->_voltArray[i]); 
    }

    msg.cell_volt = cellVolts;

    this->_publisher->publish(msg);

    cellVolts.clear();
}

void BMSDataNode::getData(void)
{
    std::string ampSerialOutput;
    std::string voltSerialOutput;
    int fileDesc;

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

    this->parse(ampSerialOutput, this->_ampArray, AMP_DATA_TYPES);
    this->parse(voltSerialOutput, this->_voltArray, VOLT_DATA_TYPES);

    if (close(fileDesc) == -1)
    {
        std::string errorMsg = "unable to close the fileDesc:" + std::string(strerror(errno));
        RCLCPP_WARN(this->get_logger(), errorMsg.c_str());
    }
}

void BMSDataNode::parse(std::string rawOutput, int16_t dataArray[], uint16_t arraySize)
{
    uint16_t indexGarb = 0;

    while(indexGarb < rawOutput.size() && rawOutput[indexGarb] != '=')
    {
        indexGarb++;
    }

    rawOutput = rawOutput.substr(indexGarb + 1);

    for (uint16_t i = 0; i < arraySize; i++)
    {
        uint16_t index = 0;

        while (index < rawOutput.size() && rawOutput[index] != ':')
        {
            index++;
        }
        dataArray[i] = std::stoi(rawOutput.substr(0, index));

        if (i < arraySize - 1)
        {
            rawOutput = rawOutput.substr(index + 1);
        }
    }
}


