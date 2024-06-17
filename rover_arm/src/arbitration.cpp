#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "rover_msgs/msg/joy.hpp"
#include "rover_msgs/msg/propulsion_motor.hpp"
#include "rover_msgs/msg/joy_demux_status.hpp"

#include "rover_msgs/srv/drive_train_arbitration.hpp"

// Class definition
class Arbitration : public rclcpp::Node
{
public:
    Arbitration();
    ~Arbitration() {}

private:
    void cbTimerSendCmd();

    void cbBaseWatchdog();
    void cbRoverWatchdog();

    void cbPropulsionCmd(const rover_msgs::msg::PropulsionMotor msg_);
    void cbBaseHr(const std_msgs::msg::Empty msg_);
    void cbRoverHr(const std_msgs::msg::Empty msg_);

    void cbAbtr(const std::shared_ptr<rover_msgs::srv::DriveTrainArbitration::Request> request,
                std::shared_ptr<rover_msgs::srv::DriveTrainArbitration::Response> response);

    void sendCmd();

    rclcpp::Subscription<rover_msgs::msg::PropulsionMotor>::SharedPtr _subMotorCmd;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr _subBaseHr;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr _subRoverHr;

    rclcpp::Publisher<rover_msgs::msg::PropulsionMotor>::SharedPtr _pubAbtr;

    rclcpp::Service<rover_msgs::srv::DriveTrainArbitration>::SharedPtr _srvControlDemux;

    rover_msgs::srv::DriveTrainArbitration::Request _arbitrationRequest;
    rover_msgs::srv::DriveTrainArbitration::Response _arbitrationResponse;

    rover_msgs::msg::PropulsionMotor _cmdTeleop;

    std_msgs::msg::Empty _hrBase;
    std_msgs::msg::Empty _hrRover;

    rclcpp::TimerBase::SharedPtr _timerSendCmd;
    rclcpp::TimerBase::SharedPtr _watchdogBase;
    rclcpp::TimerBase::SharedPtr _watchdogRover;

    bool _baseHrLost = false;
    bool _roverHrLost = false;
};

Arbitration::Arbitration() : Node("arbitration")
{
    _subBaseHr = this->create_subscription<std_msgs::msg::Empty>("/base/heartbeat",
                                                                 1,
                                                                 std::bind(&Arbitration::cbBaseHr, this, std::placeholders::_1));
    _subRoverHr = this->create_subscription<std_msgs::msg::Empty>("/rover/heartbeat",
                                                                  1,
                                                                  std::bind(&Arbitration::cbRoverHr, this, std::placeholders::_1));

    _watchdogBase = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&Arbitration::cbBaseWatchdog, this));
    _watchdogRover = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&Arbitration::cbRoverWatchdog, this));
    _timerSendCmd = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&Arbitration::cbTimerSendCmd, this));

    // Starting by default at drive_train for CRQRC, should be set when GUI
    // opens instead
}

void Arbitration::cbTimerSendCmd()
{
    sendCmd();
}

void Arbitration::cbBaseHr(const std_msgs::msg::Empty msg_)
{
    _hrBase = msg_;
    _baseHrLost = false;
    _watchdogBase->reset();
}

void Arbitration::cbRoverHr(const std_msgs::msg::Empty msg_)
{
    _hrRover = msg_;
    _roverHrLost = false;
    _watchdogRover->reset();
}

void Arbitration::cbPropulsionCmd(const rover_msgs::msg::PropulsionMotor msg_)
{
    _cmdTeleop = msg_;
}

void Arbitration::cbBaseWatchdog()
{
    _baseHrLost = true;
}

void Arbitration::cbRoverWatchdog()
{
    _roverHrLost = true;
}

void Arbitration::sendCmd()
{
    if (_baseHrLost || _roverHrLost)
    {
        rover_msgs::msg::PropulsionMotor _zeroCmd;

        for (int i = 0; i < 4; ++i)
        {
            _zeroCmd.enable[i] = false;
            _zeroCmd.target_speed[i] = 0.0;
            _zeroCmd.current_speed[i] = 0.0;
            _zeroCmd.close_loop[i] = false;
        }

        _pubAbtr->publish(_zeroCmd);
    }
    else if (_arbitrationRequest.target_arbitration == rover_msgs::srv::DriveTrainArbitration_Request::TELEOP)
    {
        _pubAbtr->publish(_cmdTeleop);
        _arbitrationResponse.current_arbitration = rover_msgs::srv::DriveTrainArbitration_Request::TELEOP;
    }

    else
    {
        rover_msgs::msg::PropulsionMotor _zeroCmd;
        for (int i = 0; i < 4; ++i)
        {
            _zeroCmd.enable[i] = false;
            _zeroCmd.target_speed[i] = 0.0;
            _zeroCmd.current_speed[i] = 0.0;
            _zeroCmd.close_loop[i] = false;
        }
        _pubAbtr->publish(_zeroCmd);
        _arbitrationResponse.current_arbitration = rover_msgs::srv::DriveTrainArbitration_Request::NONE;
    }
}

void Arbitration::cbAbtr(const std::shared_ptr<rover_msgs::srv::DriveTrainArbitration::Request> request_,
                         std::shared_ptr<rover_msgs::srv::DriveTrainArbitration::Response> response_)
{
    _arbitrationRequest = *request_;
    *response_ = _arbitrationResponse;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Arbitration>());
    rclcpp::shutdown();

    return 0;
}
