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
    void cbTimerSendCmd()
    {
        sendCmd();
    }
    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Subscription<rover_msgs::msg::AntennaCmd>::SharedPtr _sub_jog;
    rclcpp::Subscription<rover_msgs::msg::AntennaCmd>::SharedPtr _sub_auto;
    rclcpp::Publisher<rover_msgs::msg::AntennaCmd>::SharedPtr _pub_abtr;
    rclcpp::Service<rover_msgs::srv::AntennaArbitration>::SharedPtr _serviceArbitrationMode;

    void callbackJog(const rover_msgs::msg::AntennaCmd msg_);
    void callbackAuto(const rover_msgs::msg::AntennaCmd msg_);
    void cbSetAbtr(const std::shared_ptr<rover_msgs::srv::AntennaArbitration::Request> request_, 
                    std::shared_ptr<rover_msgs::srv::AntennaArbitration::Response> response_);
    void sendCmd();

    rover_msgs::msg::AntennaCmd gCmdJog;
    rover_msgs::msg::AntennaCmd gCmdAuto;
    rover_msgs::srv::AntennaArbitration::Response gArbitrationResponse;
    rover_msgs::srv::AntennaArbitration::Request gArbitrationRequest;
    
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

    _serviceArbitrationMode = this->create_service<rover_msgs::srv::AntennaArbitration>("/base/antenna/set_arbitration", std::bind(&Arbitration::cbSetAbtr, this, std::placeholders::_1, std::placeholders::_2));

    _timer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Arbitration::cbTimerSendCmd, this));

}

void Arbitration::callbackJog(const rover_msgs::msg::AntennaCmd msg_)
{
    gCmdJog = msg_;
}

void Arbitration::callbackAuto(const rover_msgs::msg::AntennaCmd msg_)
{
    gCmdAuto = msg_;
}

void Arbitration::sendCmd()
{
    rover_msgs::msg::AntennaCmd cmd_abtr;
    if (gArbitrationRequest.target_arbitration == rover_msgs::srv::AntennaArbitration_Request::TELEOP)
    {
        _pub_abtr->publish(gCmdJog);
        gArbitrationResponse.current_arbitration = rover_msgs::srv::AntennaArbitration_Request::TELEOP;
    }
    else if (gArbitrationRequest.target_arbitration == rover_msgs::srv::AntennaArbitration_Request::AUTONOMUS)
    {
        _pub_abtr->publish(gCmdAuto);
        gArbitrationResponse.current_arbitration = rover_msgs::srv::AntennaArbitration_Request::AUTONOMUS;
    }
    else
    {
        cmd_abtr.speed = 0.0f;
        cmd_abtr.status = false;
        _pub_abtr->publish(cmd_abtr);
        gArbitrationResponse.current_arbitration = rover_msgs::srv::AntennaArbitration_Request::NOT_MOVING;
    }
}

void Arbitration::cbSetAbtr(const std::shared_ptr<rover_msgs::srv::AntennaArbitration::Request> request_, 
                    std::shared_ptr<rover_msgs::srv::AntennaArbitration::Response> response_)
{
    gArbitrationRequest = *request_;
    *response_ = gArbitrationResponse;
}