#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/joy.hpp"
#include "rover_msgs/msg/antenna_cmd.hpp"
#include "rovus_lib/moving_average.hpp"
#include "rovus_lib/macros.h"
#include "rovus_lib/timer.hpp"

#define COEFF_NB 5

class JogAntenna : public rclcpp::Node
{
public:
    JogAntenna();
    ~JogAntenna() {}

private:
    void callbackJoy(const rover_msgs::msg::Joy msg_);
    void cbTimerJogCmd();

    rclcpp::Subscription<rover_msgs::msg::Joy>::SharedPtr _sub_joy;
    rclcpp::Publisher<rover_msgs::msg::AntennaCmd>::SharedPtr _pub_jog;
    rclcpp::TimerBase::SharedPtr _timer_JogCmd;

    float _l1 = 0.0;
    float _r1 = 0.0;
    bool _joyMsgReceived = false;

    rclcpp::Parameter _paramMaxSpeed;
    rclcpp::Parameter _paramCoeffNb;
    MovingAverage<float, COEFF_NB> _jogAverage_r1 = MovingAverage<float, COEFF_NB>(0.0f);
    MovingAverage<float, COEFF_NB> _jogAverage_l1 = MovingAverage<float, COEFF_NB>(0.0f);
    Timer<uint64_t, millis> _timer_joyMsg = Timer<uint64_t, millis>(250);
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
    _sub_joy = this->create_subscription<rover_msgs::msg::Joy>("/base/antenna/joy",
                                                               1,
                                                               [this](const rover_msgs::msg::Joy msg_)
                                                               { callbackJoy(msg_); });

    _pub_jog = this->create_publisher<rover_msgs::msg::AntennaCmd>("/base/antenna/cmd/in/teleop", 1);

    _timer_JogCmd = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&JogAntenna::cbTimerJogCmd, this));

    this->declare_parameter<float>("max_speed", PI / 2.0f);
    _paramMaxSpeed = this->get_parameter("max_speed");
}

void JogAntenna::cbTimerJogCmd()
{
    rover_msgs::msg::AntennaCmd jogCmd;

    _jogAverage_l1.addValue(_l1);

    _jogAverage_r1.addValue(_r1);

    jogCmd.enable = true;

    if (_joyMsgReceived)
    {

        _jogAverage_l1.addValue(_l1);

        _jogAverage_r1.addValue(_r1);

        jogCmd.enable = true;

        if (_jogAverage_l1.getAverage() == 0.0f && _jogAverage_r1.getAverage() == 0.0f)
        {
            jogCmd.speed = 0.0f;
            jogCmd.enable = false;
        }
        else if (_jogAverage_l1.getAverage() != 0.0f && _jogAverage_r1.getAverage() == 0.0f)
        {
            jogCmd.speed = -_jogAverage_l1.getAverage() * _paramMaxSpeed.as_double();
        }
        else if (_jogAverage_l1.getAverage() == 0.0f && _jogAverage_r1.getAverage() != 0.0f)
        {
            jogCmd.speed = _jogAverage_r1.getAverage() * _paramMaxSpeed.as_double();
        }
        else
        {
            jogCmd.speed = 0.0f;
            jogCmd.enable = false;
        }
        jogCmd.enable = true;
        _pub_jog->publish(jogCmd);
    }
    else
    {
        RCLCPP_WARN(LOGGER, "No joy message is received");
        jogCmd.enable = false;
        jogCmd.speed = 0.0;
        _pub_jog->publish(jogCmd);
    }
}

void JogAntenna::callbackJoy(const rover_msgs::msg::Joy msg_)
{
    _l1 = msg_.joy_data[rover_msgs::msg::Joy::L1];
    _r1 = msg_.joy_data[rover_msgs::msg::Joy::R1];
    ;

    _joyMsgReceived = true;
    _timer_joyMsg.init(250);
}