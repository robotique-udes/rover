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
    Node("bms_data")
{
    _publisher = this->create_publisher<rover_msgs::msg::BmsData>(TOPIC_BMS_DATA, QOS_DEFAULT);

    _timer_publisher = this->create_wall_timer(std::chrono::milliseconds(DELAY_PUBLISHER_MS),
                                                [this](void)
                                                {
                                                    this->callbackBMSData();
                                                });
}

void BMSDataNode::callbackBMSData()
{
    
}