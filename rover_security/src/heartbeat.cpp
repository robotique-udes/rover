#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"

int main(int argc, char *argv[])
{
    uint8_t heartbeat_frequency = 1u;
    
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("base_heartbeat");
    node->declare_parameter<uint8_t>("heartbeat_frequency", 1);

    heartbeat_frequency = static_cast<uint8_t>(node->get_parameter("heartbeat_frequency").as_int());
    
    if (heartbeat_frequency == 1u)
    {
        RCLCPP_WARN(node->get_logger(), "Failed to get frequency param, retrying...");
    }
    else if (node->get_parameter("heartbeat_frequency").as_int() <= 0)
    {
        RCLCPP_WARN(node->get_logger(), "Parameter does not fit expected format");
    }
    
    RCLCPP_INFO(node->get_logger(), "Starting heartbeat at %u Hz", heartbeat_frequency);

    rclcpp::Publisher<std_msgs:: msg::Empty>::SharedPtr pub_heartbeat = node->create_publisher<std_msgs::msg::Empty>("heartbeat", 1);
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
