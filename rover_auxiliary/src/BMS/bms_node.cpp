#include "bms_node.hpp"



int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BMSDataNode>(argc, argv);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

BMSDataNode::BMSDataNode(int argc, char** argv):
    Node("bms_info")
{
    _publisher = this->create_publisher<rover_msgs::msg::BmsData>(TOPIC_BMS_DATA, QOS_DEFAULT);

    _timer_publisher = this->create_wall_timer(std::chrono::milliseconds(DELAY_PUBLISHER_MS),
                                                [this](void)
                                                {
                                                    this->callbackBMSData();
                                                });
}

int BMSDataNode::getData()
{
    _cellVolt.reserve(MAX_CELL);
    std::ifstream file(DATA_FILE_PATH);
    std::string stringAmp;
    char number;

    if (!file.is_open())
    {
        std::cout << "Error" << std::endl;
        return 1;
    }

    while(file.get() != '=')
    {}

    file.get(number);
    while(number != ' ' && number != '\n')
    {
            stringAmp += number;
            file.get(number);
    }

    _batteryAmps = std::stoi(stringAmp);
    std::cout << _batteryAmps << std::endl;

    while(file.get() != '=')
    {}

    for(uint16_t indexCell=0;indexCell<MAX_CELL;indexCell++)
    {
        std::string stringVolt;

        file.get(number);

        while(number != ':' && number != ' ' && number != '\n')
        {
            stringVolt += number;
            file.get(number);
        }

        _cellVolt[indexCell] = static_cast<uint16_t>(std::stoi(stringVolt));
        std::cout << _cellVolt[indexCell] << std::endl;
    }

    file.close();

    return 0;
}

void BMSDataNode::callbackBMSData()
{
    rover_msgs::msg::BmsData msg;

    getData();

    msg.battery_amps = _batteryAmps;
    
    for(uint16_t index=0;index<MAX_CELL;index++)
    {
        const auto& cellV = _cellVolt[index];
        
    }
}