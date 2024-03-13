#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "rover_msgs/msg/joy.hpp"
#include <map>
#include "rover_msgs/msg/joy_demux_status.hpp"

class Teleop : public rclcpp::Node
{
private:
    rclcpp::Subscription<rover_msgs::msg::Joy>::SharedPtr _sub_joy_formatted;
    rclcpp::TimerBase::SharedPtr _timer;

public:
    Teleop();
};

Teleop::Teleop() : Node("teleop")
{
                                                                        
    this->declare_parameter<float>("speed_factor_crawler", 0.01);
    this->declare_parameter<float>("speed_factor_normal", 0.25);
    this->declare_parameter<float>("speed_factor_turbo", 1.0);
    this->declare_parameter<float>("smallest_radius", 0.30);
}

int main(int argc, char *argv[])
{
    float speed_factor_crawler;
    float speed_factor_normal;
    float speed_factor_turbo;
    float smallest_radius;
    
    Teleop::SharedPtr node = Teleop::make_shared("node_drive_train");

    speed_factor_crawler = static_cast<float>(node->get_parameter("speed_factor_crawler").as_double());
    speed_factor_normal = static_cast<float>(node->get_parameter("speed_factor_turbo").as_double());
    speed_factor_turbo = static_cast<float>(node->get_parameter("speed_factor_turbo").as_double());
    smallest_radius = static_cast<float>(node->get_parameter("smallest_radius").as_double());    

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Crawler %f", speed_factor_crawler);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "normal %f", speed_factor_normal);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "turbo %f", speed_factor_turbo);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "radius %f", smallest_radius);

    rclcpp::init(argc, argv);
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}