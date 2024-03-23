#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/antenna_cmd.hpp"
#include "rover_msgs/srv/antenna_arbitration.hpp"

class Arbitration : public rclcpp::Node
{
public:
    Arbitration();
    ~Arbitration() {}

private:
    void cbTimerSendCmd();
    void callbackJog(const rover_msgs::msg::AntennaCmd msg_);
    void callbackAuto(const rover_msgs::msg::AntennaCmd msg_);
    void cbSetAbtr(const std::shared_ptr<rover_msgs::srv::AntennaArbitration::Request> request_, 
                    std::shared_ptr<rover_msgs::srv::AntennaArbitration::Response> response_);
    void sendCmd();
  
    rclcpp::TimerBase::SharedPtr _timer_SendCmd;
    rclcpp::Subscription<rover_msgs::msg::AntennaCmd>::SharedPtr _sub_jog;
    rclcpp::Subscription<rover_msgs::msg::AntennaCmd>::SharedPtr _sub_auto;
    rclcpp::Publisher<rover_msgs::msg::AntennaCmd>::SharedPtr _pub_abtr;
    rclcpp::Service<rover_msgs::srv::AntennaArbitration>::SharedPtr _service_ArbitrationMode;
    
    rover_msgs::msg::AntennaCmd _cmdJog;
    rover_msgs::msg::AntennaCmd _cmdAuto;

    rover_msgs::srv::AntennaArbitration::Response _arbitrationResponse;
    rover_msgs::srv::AntennaArbitration::Request _arbitrationRequest;
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

    _service_ArbitrationMode = this->create_service<rover_msgs::srv::AntennaArbitration>("/base/antenna/set_arbitration", std::bind(&Arbitration::cbSetAbtr, this, std::placeholders::_1, std::placeholders::_2));

    _timer_SendCmd = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Arbitration::cbTimerSendCmd, this));
}

void Arbitration::cbTimerSendCmd()
{
    sendCmd();
}

void Arbitration::callbackJog(const rover_msgs::msg::AntennaCmd msg_)
{
    _cmdJog = msg_;
}

void Arbitration::callbackAuto(const rover_msgs::msg::AntennaCmd msg_)
{
    _cmdAuto = msg_;
}

void Arbitration::sendCmd()
{
    rover_msgs::msg::AntennaCmd cmdAbtr;
    if (_arbitrationRequest.target_arbitration == rover_msgs::srv::AntennaArbitration_Request::TELEOP)
    {
        _pub_abtr->publish(_cmdJog);
        _arbitrationResponse.current_arbitration = rover_msgs::srv::AntennaArbitration_Request::TELEOP;
    }
    else if (_arbitrationRequest.target_arbitration == rover_msgs::srv::AntennaArbitration_Request::AUTONOMUS)
    {
        _pub_abtr->publish(_cmdAuto);
        _arbitrationResponse.current_arbitration = rover_msgs::srv::AntennaArbitration_Request::AUTONOMUS;
    }
    else
    {
        cmdAbtr.speed = 0.0f;
        cmdAbtr.enable = false;
        _pub_abtr->publish(cmdAbtr);
        _arbitrationResponse.current_arbitration = rover_msgs::srv::AntennaArbitration_Request::NOT_MOVING;
    }
}

void Arbitration::cbSetAbtr(const std::shared_ptr<rover_msgs::srv::AntennaArbitration::Request> request_, 
                    std::shared_ptr<rover_msgs::srv::AntennaArbitration::Response> response_)
{
    _arbitrationRequest = *request_;
    *response_ = _arbitrationResponse;
}