#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "rover_msgs/msg/joy.hpp"
#include "rover_msgs/msg/propulsion_motor.hpp"
#include "rover_msgs/msg/joy_demux_status.hpp"

#include "rover_msgs/srv/drive_train_arbitration.hpp"

// Class definition
class ClientUDPAntenna : public rclcpp::Node
{
public:
    ClientUDPAntenna();
    ~ClientUDPAntenna() {}

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

ClientUDPAntenna::ClientUDPAntenna() : Node("arbitration")
{
    _subBaseHr = this->create_subscription<std_msgs::msg::Empty>("/base/heartbeat",
                                                                 1,
                                                                 std::bind(&ClientUDPAntenna::cbBaseHr, this, std::placeholders::_1));
    _subRoverHr = this->create_subscription<std_msgs::msg::Empty>("/rover/heartbeat",
                                                                  1,
                                                                  std::bind(&ClientUDPAntenna::cbRoverHr, this, std::placeholders::_1));
    _subMotorCmd = this->create_subscription<rover_msgs::msg::PropulsionMotor>("/rover/drive_train/cmd/in/teleop",
                                                                               1,
                                                                               std::bind(&ClientUDPAntenna::cbPropulsionCmd, this, std::placeholders::_1));

    _pubAbtr = this->create_publisher<rover_msgs::msg::PropulsionMotor>("/rover/drive_train/cmd/out/motors", 1);

    _srvControlDemux = this->create_service<rover_msgs::srv::DriveTrainArbitration>("demux_control_cmd", std::bind(&ClientUDPAntenna::cbAbtr, this, std::placeholders::_1, std::placeholders::_2));

    _watchdogBase = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&ClientUDPAntenna::cbBaseWatchdog, this));
    _watchdogRover = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&ClientUDPAntenna::cbRoverWatchdog, this));
    _timerSendCmd = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&ClientUDPAntenna::cbTimerSendCmd, this));

    // Starting by default at drive_train for CRQRC, should be set when GUI
    // opens instead
    _arbitrationRequest.target_arbitration = rover_msgs::srv::DriveTrainArbitration_Request::TELEOP;
}

void ClientUDPAntenna::cbTimerSendCmd()
{
    sendCmd();
}

void ClientUDPAntenna::cbBaseHr(const std_msgs::msg::Empty msg_)
{
    _hrBase = msg_;
    _baseHrLost = false;
    _watchdogBase->reset();
}

void ClientUDPAntenna::cbRoverHr(const std_msgs::msg::Empty msg_)
{
    _hrRover = msg_;
    _roverHrLost = false;
    _watchdogRover->reset();
}

void ClientUDPAntenna::cbPropulsionCmd(const rover_msgs::msg::PropulsionMotor msg_)
{
    _cmdTeleop = msg_;
}

void ClientUDPAntenna::cbBaseWatchdog()
{
    _baseHrLost = true;
}

void ClientUDPAntenna::cbRoverWatchdog()
{
    _roverHrLost = true;
}

void ClientUDPAntenna::sendCmd()
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

void ClientUDPAntenna::cbAbtr(const std::shared_ptr<rover_msgs::srv::DriveTrainArbitration::Request> request_,
                         std::shared_ptr<rover_msgs::srv::DriveTrainArbitration::Response> response_)
{
    _arbitrationRequest = *request_;
    *response_ = _arbitrationResponse;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ClientUDPAntenna>());
    rclcpp::shutdown();

    return 0;
}
