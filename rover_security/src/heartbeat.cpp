#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("base_heartbeat");
    node -> declare_parameter<uint8_t>("heartbeat_frequency", 2u);

    uint8_t heartbeat_frequency = 1u;

    //bool _flag = false;
    
    if (!node->get_parameter_or("heartbeat_frequency", heartbeat_frequency, heartbeat_frequency))
    {
        RCLCPP_WARN(node->get_logger(), "Failed to get frequency param, retrying...");
        //_flag = true;
    }

    RCLCPP_INFO(node->get_logger(), "Starting heartbeat at %u Hz", heartbeat_frequency);

    auto pub_heartbeat = node->create_publisher<std_msgs::msg::Empty>("base_heartbeat", 1);
    rclcpp::Rate timer(heartbeat_frequency);

    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        std_msgs::msg::Empty msg;
        pub_heartbeat->publish(msg);
        timer.sleep();
    }

    rclcpp::shutdown();

    return 0;
}
