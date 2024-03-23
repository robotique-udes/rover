#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/antenna_cmd.hpp"
#include "rover_msgs/srv/antenna_arbitration.hpp"

using namespace std::chrono_literals;

class Arbitration : public rclcpp::Node
{
public:
    Arbitration();
    ~Arbitration() {}

private:
    void timer_callback()
    {
        RCLCPP_INFO(rclcpp::get_logger("timer_cb"), "timer_callback");
        sendCmd();
        
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<rover_msgs::msg::AntennaCmd>::SharedPtr _sub_jog;
    rclcpp::Subscription<rover_msgs::msg::AntennaCmd>::SharedPtr _sub_auto;
    rclcpp::Publisher<rover_msgs::msg::AntennaCmd>::SharedPtr _pub_abtr;
    rclcpp::Service<rover_msgs::srv::AntennaArbitration>::SharedPtr _service;

    void callbackJog(const rover_msgs::msg::AntennaCmd msg);
    void callbackAuto(const rover_msgs::msg::AntennaCmd msg);
    void cbSetAbtr(const std::shared_ptr<rover_msgs::srv::AntennaArbitration::Request> request, 
                    std::shared_ptr<rover_msgs::srv::AntennaArbitration::Response> response);
    void sendCmd();

    rover_msgs::msg::AntennaCmd cmd_jog;
    rover_msgs::msg::AntennaCmd cmd_auto;
    rover_msgs::srv::AntennaArbitration::Response abtr_response;
    rover_msgs::srv::AntennaArbitration::Request abtr_request;
    
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

    _service = this->create_service<rover_msgs::srv::AntennaArbitration>("/base/antenna/set_arbitration", std::bind(&Arbitration::cbSetAbtr, this, std::placeholders::_1, std::placeholders::_2)); ///base/antenna/set_arbitration

    timer_ = this->create_wall_timer(100ms, std::bind(&Arbitration::timer_callback, this));

}

void Arbitration::callbackJog(const rover_msgs::msg::AntennaCmd msg)
{
    cmd_jog = msg;
}

void Arbitration::callbackAuto(const rover_msgs::msg::AntennaCmd msg)
{
    cmd_auto = msg;
}

void Arbitration::sendCmd()
{
    rover_msgs::msg::AntennaCmd cmd_abtr;
    if (abtr_request.abtr == abtr_request.TELEOP)
    {
        _pub_abtr->publish(cmd_jog);
        abtr_response.abtr_mode = abtr_request.TELEOP;
        RCLCPP_INFO(rclcpp::get_logger("abtr_mode"), "abtr_mode %d", abtr_request.abtr);
    }
    else if (abtr_request.abtr == abtr_request.AUTONOMUS)
    {
        _pub_abtr->publish(cmd_auto);
        abtr_response.abtr_mode = abtr_request.AUTONOMUS;
    }
    else
    {
        cmd_abtr.speed = 0.0;
        cmd_abtr.status = false;
        _pub_abtr->publish(cmd_abtr);
        abtr_response.abtr_mode = abtr_request.NOT_MOVING;
    }
}

void Arbitration::cbSetAbtr(const std::shared_ptr<rover_msgs::srv::AntennaArbitration::Request> request, 
                    std::shared_ptr<rover_msgs::srv::AntennaArbitration::Response> response)
{
    abtr_request = *request;
    *response = abtr_response;
}