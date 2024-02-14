#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rover_msgs/msg/joy.hpp"
#include "rover_msgs/msg/antenna_cmd.hpp"
#include "rovus_lib/macros.h"

#define PI 3.14159265359
#define MAX_SPEED 1.0*PI/180 // rad/s

using namespace std::chrono_literals;

class JogAntenna : public rclcpp::Node
{
public:
    JogAntenna();
    ~JogAntenna() {}

private:
    void timer_callback()
    {
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<rover_msgs::msg::Joy>::SharedPtr _sub_joy;
    rclcpp::Publisher<rover_msgs::msg::AntennaCmd>::SharedPtr _pub_jog;

    void callbackJoy(const rover_msgs::msg::Joy msg);
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JogAntenna>());
    rclcpp::shutdown();
    return 0;
}

JogAntenna::JogAntenna() : Node("jog_antenna")
{
    _sub_joy = this->create_subscription<rover_msgs::msg::Joy>("/joy/main/formated" /*"/base/antenna/joy"*/,
                                                               1,
                                                               [this](const rover_msgs::msg::Joy msg)
                                                               { callbackJoy(msg); });

    _pub_jog = this->create_publisher<rover_msgs::msg::AntennaCmd>("/base/antenna/cmd/in/teleop", 1);
    timer_ = this->create_wall_timer(500ms, std::bind(&JogAntenna::timer_callback, this));
}

void JogAntenna::callbackJoy(const rover_msgs::msg::Joy msg)
{
    rover_msgs::msg::AntennaCmd jog_cmd;
    jog_cmd.status = true;
    if (msg.l2 != 0.0 && msg.r2 == 0.0)
    {   
        jog_cmd.speed = -msg.l2 * MAX_SPEED;
    }
    else if (msg.r2 != 0.0 && msg.l2 == 0.0)
    {
        jog_cmd.speed = msg.r2 * MAX_SPEED;
    }
    else
    {
        jog_cmd.speed = 0.0;
    }
    _pub_jog->publish(jog_cmd);
}