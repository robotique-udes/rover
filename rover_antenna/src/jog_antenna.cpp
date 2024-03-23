#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/joy.hpp"
#include "rover_msgs/msg/antenna_cmd.hpp"
#include "rovus_lib/moving_average.hpp"
#include "rovus_lib/macros.h"

#define COEFF_NB 10

class JogAntenna : public rclcpp::Node
{
public:
    JogAntenna();
    ~JogAntenna() {}

private:
    void callbackJoy(const rover_msgs::msg::Joy msg_);

    rclcpp::Subscription<rover_msgs::msg::Joy>::SharedPtr _sub_joy;
    rclcpp::Publisher<rover_msgs::msg::AntennaCmd>::SharedPtr _pub_jog;

    rclcpp::Parameter _paramMaxSpeed;
    rclcpp::Parameter _paramCoeffNb;
    MovingAverage<float, COEFF_NB> _jogAverage_r1 = MovingAverage<float, COEFF_NB>(0.0f);
    MovingAverage<float, COEFF_NB> _jogAverage_l1 = MovingAverage<float, COEFF_NB>(0.0f);
    
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
    _sub_joy = this->create_subscription<rover_msgs::msg::Joy>("/joy/main/formated",
                                                               1,
                                                               [this](const rover_msgs::msg::Joy msg_)
                                                               { callbackJoy(msg_); });

    _pub_jog = this->create_publisher<rover_msgs::msg::AntennaCmd>("/base/antenna/cmd/in/teleop", 1);

    this->declare_parameter<float>("max_speed", PI/2.0f);
    _paramMaxSpeed = this->get_parameter("max_speed");
    
}

void JogAntenna::callbackJoy(const rover_msgs::msg::Joy msg_)
{
    rover_msgs::msg::AntennaCmd jogCmd;

    _jogAverage_l1.addValue(msg_.l1);
    
    _jogAverage_r1.addValue(msg_.r1);

    jogCmd.enable = true;

    if(_jogAverage_l1.getAverage() != 0.0f && _jogAverage_r1.getAverage() == 0.0f)
    {
        jogCmd.speed = -_jogAverage_l1.getAverage() * _paramMaxSpeed.as_double();
    }
    else if(_jogAverage_l1.getAverage() == 0.0f && _jogAverage_r1.getAverage() != 0.0f)
    {
        jogCmd.speed = _jogAverage_r1.getAverage() * _paramMaxSpeed.as_double();
    }
    else
    {
        jogCmd.speed = 0.0f;
    }

    _pub_jog->publish(jogCmd);
}