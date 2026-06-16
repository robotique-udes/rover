#include "bms_node.hpp"

int main(int argc_, char** argv_)
{
    rclcpp::init(argc_, argv_);
    std::shared_ptr<BMSDataNode> node = std::make_shared<BMSDataNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

BMSDataNode::BMSDataNode():
    Node("bms_info"),
    _terminal("/dev/ttyACM0")
{
    _publisher = create_publisher<rover_msgs::msg::BmsData>(TOPIC_BMS_DATA, QOS_DEFAULT);

    _timer_publisher = create_wall_timer(std::chrono::milliseconds(DELAY_PUBLISHER_MS),
                                         [this](void)
                                         {
                                             this->callbackBMSData();
                                         });
}

void BMSDataNode::callbackBMSData(void)
{
    rover_msgs::msg::BmsData msg;

    this->getData();

    msg.battery_amps = _ampArray[AmpIndexType::BATTERY_AMPS];

    msg.cell_volt.resize(CELL_VOLT_END - CELL_VOLT_START);

    for (uint16_t i = VoltIndexType::CELL_VOLT_START; i < VoltIndexType::CELL_VOLT_END; i++)
    {
        msg.cell_volt[i - CELL_VOLT_START] = _voltArray[i];
    }

    _publisher->publish(msg);
}

void BMSDataNode::getData(void)
{
    std::string ampSerialOutput;
    std::string voltSerialOutput;

    _terminal.serialWrite("?A\r");
    ampSerialOutput = _terminal.serialRead();
    _terminal.serialWrite("?V\r");
    voltSerialOutput = _terminal.serialRead();

    this->parse(ampSerialOutput, _ampArray, AMP_DATA_TYPES);
    this->parse(voltSerialOutput, _voltArray, VOLT_DATA_TYPES);
}

void BMSDataNode::parse(const std::string rawOutput_, uint16_t dataArray_[], uint16_t arraySize_)
{
    std::string_view view = rawOutput_;

    size_t startPos = view.find('=');
    if (startPos == std::string_view::npos)
    {
        RCLCPP_WARN(this->get_logger(), "Command to retrieve data from BMS failed");
        return;
    }

    view.remove_prefix(startPos + 1);

    if (view.empty())
    {
        RCLCPP_WARN(this->get_logger(), "No data retreived from BMS command");
        return;
    }

    for (uint16_t i = 0; i < arraySize_; i++)
    {
        size_t delimPos = view.find(':');

        dataArray_[i] = std::stoi(std::string(view.substr(0, delimPos)));

        if (delimPos == std::string_view::npos)
        {
            if (i < arraySize_ - 1)
            {
                RCLCPP_WARN(this->get_logger(), "BMS's command output returned fewer data than expected");
            }
            return;
        }

        view.remove_prefix(delimPos + 1);
    }
}