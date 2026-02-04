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

    std::ifstream file(DATA_FILE_PATH);
    std::string line;
    bool skip = true;

    if (!file.is_open())
    {
        std::cout << "Error" << std::endl;
        return 1;
    }

    while(getline(file, line))
    {
        line.erase(std::remove(line.begin(), line.end(), '\r'), line.end());

        std::cout << line << std::endl;
    }

    file.close();

    return 0;
}

void BMSDataNode::callbackBMSData()
{
    rover_msgs::msg::BmsData msg;

    getData();
}