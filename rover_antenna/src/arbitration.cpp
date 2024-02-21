#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rover_msgs/msg/joy.hpp"
#include "rover_msgs/msg/antenna_cmd.hpp"
#include "rovus_lib/macros.h"

using namespace std::chrono_literals;

class Arbitration : public rclcpp::Node
{
public:
    Arbitration();
    ~Arbitration() {}

private:
    void timer_callback()
    {
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<rover_msgs::msg::AntennaCmd>::SharedPtr _sub_jog;
    rclcpp::Subscription<rover_msgs::msg::AntennaCmd>::SharedPtr _sub_auto;
    rclcpp::Publisher<rover_msgs::msg::AntennaCmd>::SharedPtr _pub_abtr;

    void callbackJog(const rover_msgs::msg::AntennaCmd msg);
    void callbackAuto(const rover_msgs::msg::AntennaCmd msg);
    void sendCmd();

    rover_msgs::msg::AntennaCmd cmd_jog;
    rover_msgs::msg::AntennaCmd cmd_auto;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Arbitration>());
    rclcpp::shutdown();
    return 0;
}

Arbitration::Arbitration() : Node("arbitration")
{
    _sub_jog = this->create_subscription<rover_msgs::msg::AntennaCmd>("/base/antenna/cmd/in/teleop",
                                                                     1,
                                                                     [this](const rover_msgs::msg::AntennaCmd msg)
                                                                     { callbackJog(msg); });
                                                                     
    _sub_auto = this->create_subscription<rover_msgs::msg::AntennaCmd>("/base/antenna/cmd/in/auto",
                                                                     1,
                                                                     [this](const rover_msgs::msg::AntennaCmd msg)
                                                                     { callbackAuto(msg); });
    _pub_abtr = this->create_publisher<rover_msgs::msg::AntennaCmd>("/base/antenna/cmd/out/goal", 1);
}

void Arbitration::callbackJog(const rover_msgs::msg::AntennaCmd msg)
{
    cmd_jog = msg;
    sendCmd();

}

void Arbitration::callbackAuto(const rover_msgs::msg::AntennaCmd msg)
{
    cmd_auto = msg;
}

void Arbitration::sendCmd()
{
    rover_msgs::msg::AntennaCmd cmd_abtr;
    // if (cmd_jog.speed != 0.0)
    // {
    //     cmd_abtr = cmd_jog;
    // }
    // else 
    // {
    //     cmd_abtr = cmd_auto;
    // }
    cmd_abtr = cmd_jog;
    RCLCPP_INFO(LOGGER, "cmd_jog : %f", cmd_jog.speed);
    _pub_abtr->publish(cmd_abtr);
}