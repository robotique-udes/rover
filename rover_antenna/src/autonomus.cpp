#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rover_msgs/msg/joy.hpp"
#include "rover_msgs/msg/antenna_cmd.hpp"
#include "rovus_lib/macros.h"

#define MAX_SPEED 1.0 // deg/s

using namespace std::chrono_literals;

class Autonomus : public rclcpp::Node
{
public:
    Autonomus();
    ~Autonomus() {}

private:
    void timer_callback()
    {
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<rover_msgs::msg::Joy>::SharedPtr _sub_gps_rover;
    rclcpp::Subscription<rover_msgs::msg::Joy>::SharedPtr _sub_gps_antenna;
    rclcpp::Publisher<rover_msgs::msg::AntennaCmd>::SharedPtr _pub_cmd;

    void callbackGPSRover(const rover_msgs::msg::Joy msg);
    void callbackGPSAntenna(const rover_msgs::msg::Joy msg);
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Autonomus>());
    rclcpp::shutdown();
    return 0;
}

Autonomus::Autonomus() : Node("autonomus")
{
    _sub_gps_rover = this->create_subscription<rover_msgs::msg::Joy>("/rover/gps/position" /*"/base/antenna/joy"*/,
                                                                     1,
                                                                     [this](const rover_msgs::msg::Joy msg)
                                                                     { callbackGPSRover(msg); });

    _sub_gps_antenna = this->create_subscription<rover_msgs::msg::Joy>("/base/antenna/gps/position" /*"/base/antenna/joy"*/,
                                                                     1,
                                                                     [this](const rover_msgs::msg::Joy msg)
                                                                     { callbackGPSAntenna(msg); });

    _pub_cmd = this->create_publisher<rover_msgs::msg::AntennaCmd>("/base/antenna/cmd/in/auto", 1);
    timer_ = this->create_wall_timer(500ms, std::bind(&Autonomus::timer_callback, this));
}

void Autonomus::callbackGPSRover(const rover_msgs::msg::Joy msg)
{

}

void Autonomus::callbackGPSAntenna(const rover_msgs::msg::Joy msg)
{

}