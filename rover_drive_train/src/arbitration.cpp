#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "rover_msgs/msg/joy.hpp"
#include "rover_msgs/msg/propulsion_motor.hpp"
#include "rover_msgs/msg/joy_demux_status.hpp"

#include "rover_msgs/srv/drive_train_arbitration.hpp"

//Class definition
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

        rclcpp::Subscription<rover_msgs::msg::PropulsionMotor>::SharedPtr _sub_motor_cmd;
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr _sub_base_heartbeat;
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr _sub_rover_heartbeat;

        rclcpp::Publisher<rover_msgs::msg::PropulsionMotor>::SharedPtr _pub_abtr;

        rclcpp::Service<rover_msgs::srv::DriveTrainArbitration>::SharedPtr _srv_control_demux;

        rover_msgs::srv::DriveTrainArbitration::Request _arbitrationRequest;
        rover_msgs::srv::DriveTrainArbitration::Response _arbitrationResponse;

        rover_msgs::msg::PropulsionMotor _cmdTeleop;

        std_msgs::msg::Empty _hrBase;
        std_msgs::msg::Empty _hrRover;

        rclcpp::TimerBase::SharedPtr _timer_SendCmd;
        rclcpp::TimerBase::SharedPtr _watchdog_base;
        rclcpp::TimerBase::SharedPtr _watchdog_rover;
        
        bool _baseHrLost = false;
        bool _roverHrLost = false;
};

Arbitration::Arbitration() : Node("arbitration")
{
    _sub_base_heartbeat = this->create_subscription<std_msgs::msg::Empty>("/base/heartbeat",
                                                                                    1,
                                                                                    std::bind(&Arbitration::cbBaseHr, this, std::placeholders:: _1));
    _sub_rover_heartbeat = this->create_subscription<std_msgs::msg::Empty>("/rover/heartbeat",
                                                                                    1,
                                                                                    std::bind(&Arbitration::cbRoverHr, this, std::placeholders:: _1));
    _sub_motor_cmd = this->create_subscription<rover_msgs::msg::PropulsionMotor>("/rover/drive_train/cmd/in/teleop",
                                                                                    1,
                                                                                    std::bind(&Arbitration::cbPropulsionCmd, this, std::placeholders::_1));

    _pub_abtr = this->create_publisher<rover_msgs::msg::PropulsionMotor>("/rover/drive_train/cmd/out/raw", 1);

    _srv_control_demux = this->create_service<rover_msgs::srv::DriveTrainArbitration>("demux_control_cmd", std::bind(&Arbitration::cbAbtr, this, std::placeholders::_1, std::placeholders::_2));

    _watchdog_base = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&Arbitration::cbBaseWatchdog, this));
    _watchdog_rover = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&Arbitration::cbRoverWatchdog, this));
    _timer_SendCmd = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&Arbitration::cbTimerSendCmd, this));
}

void Arbitration::cbTimerSendCmd()
{
    sendCmd();
}

void Arbitration::cbBaseHr(const std_msgs::msg::Empty msg_)
{
    _hrBase = msg_;
    _baseHrLost = false;
    _watchdog_base->reset();
}

void Arbitration::cbRoverHr(const std_msgs::msg::Empty msg_)
{
    _hrRover = msg_;
    _roverHrLost = false;
    _watchdog_rover->reset();
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
    if(_baseHrLost || _roverHrLost)
    {
        rover_msgs::msg::PropulsionMotor _zeroCmd;

        for(int i = 0; i < 4; ++i)
        {
            _zeroCmd.enable[i] = false;
            _zeroCmd.target_speed[i] = 0.0;
            _zeroCmd.current_speed[i] = 0.0;
            _zeroCmd.close_loop[i] = false;
        }

        _pub_abtr->publish(_zeroCmd);
    }
    else if (_arbitrationRequest.target_arbitration == rover_msgs::srv::DriveTrainArbitration_Request::TELEOP)
    {

        _pub_abtr->publish(_cmdTeleop);
        _arbitrationResponse.current_arbitration = rover_msgs::srv::DriveTrainArbitration_Request::TELEOP;
    }

    else
    {
        rover_msgs::msg::PropulsionMotor _zeroCmd;
        for(int i = 0; i < 4; ++i)
        {
            _zeroCmd.enable[i] = false;
            _zeroCmd.target_speed[i] = 0.0;
            _zeroCmd.current_speed[i] = 0.0;
            _zeroCmd.close_loop[i] = false;
        }
        _pub_abtr->publish(_zeroCmd);
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
