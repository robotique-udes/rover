#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "rover_msgs/msg/joy.hpp"

using namespace std::placeholders;

class Teleop : public rclcpp::Node
{
private:
    rclcpp::Subscription<rover_msgs::msg::Joy>::SharedPtr _sub_joy_formatted;
    rclcpp::TimerBase::SharedPtr _timer;

    void callbackTeleop(const rover_msgs::msg::Joy &msg);
    void callbackTimer();

public:
    Teleop();
};

int main(int argc, char *argv[])
{
    float speed_factor_crawler = 0.1;
    float speed_factor_normal = 0.1;
    float speed_factor_turbo = 0.1;
    float smallest_radius = 0.1;
    
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

Teleop::Teleop() : Node("teleop")
{
    _sub_joy_formatted = this->create_subscription<rover_msgs::msg::Joy>("main_formatted",
                                                                         1,
                                                                          [this](const rover_msgs::msg::Joy msg)
                                                                        {callbackTeleop(msg); });

    _timer = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&Teleop::callbackTimer, this));
                                                                         
    this->declare_parameter<float>("speed_factor_crawler", 0.01);
    this->declare_parameter<float>("speed_factor_normal", 0.25);
    this->declare_parameter<float>("speed_factor_turbo", 1.0);
    this->declare_parameter<float>("smallest_radius", 0.30);
}

void Teleop::callbackTeleop(const rover_msgs::msg::Joy &msg)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Teleop is active");
}

void Teleop::callbackTimer()
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Timer is active");
}