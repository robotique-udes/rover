#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/drivetrain_arbitration.hpp"
#include "rover_msgs/msg/joy.hpp"
#include "rover_msgs/msg/joy_demux_status.hpp"
#include "rover_msgs/msg/propulsion_motor.hpp"
#include "std_msgs/msg/empty.hpp"

#include "rover_msgs/srv/drive_train_arbitration.hpp"
#include "rovus_lib/macros.h"

class Arbitration : public rclcpp::Node
{
  public:
    Arbitration();
    ~Arbitration() {}

  private:
    void cbTimerSendCmd();
    void cbTimerSendStatus();
    void watchdog(bool* LostHB);
    void cbPropulsionCmd(const rover_msgs::msg::PropulsionMotor msg_);
    void cbHB(const std_msgs::msg::Empty msg_, bool* _HBLostVar, rclcpp::TimerBase::SharedPtr _HBWatchdogTimer);
    void cbAbtr(const std::shared_ptr<rover_msgs::srv::DriveTrainArbitration::Request> request,
                std::shared_ptr<rover_msgs::srv::DriveTrainArbitration::Response> response);
    void sendCmd();

    rclcpp::Subscription<rover_msgs::msg::PropulsionMotor>::SharedPtr _subMotorCmdTeleop;
    rclcpp::Subscription<rover_msgs::msg::PropulsionMotor>::SharedPtr _subMotorCmdAuto;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr _subBaseHr;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr _subRoverHr;

    rclcpp::Publisher<rover_msgs::msg::PropulsionMotor>::SharedPtr _pubCmd;

    rclcpp::Service<rover_msgs::srv::DriveTrainArbitration>::SharedPtr _srvControlDemux;
    rclcpp::Publisher<rover_msgs::msg::DrivetrainArbitration>::SharedPtr _pubArbitrationStatus;

    rover_msgs::msg::DrivetrainArbitration _arbitration;

    rover_msgs::msg::PropulsionMotor _zeroCmd;
    rover_msgs::msg::PropulsionMotor _cmdTeleop;

    rclcpp::TimerBase::SharedPtr _timerSendCmd;
    rclcpp::TimerBase::SharedPtr _timerSendStatus;
    rclcpp::TimerBase::SharedPtr _watchdogBase;
    rclcpp::TimerBase::SharedPtr _watchdogRover;

    bool _baseHBLost = false;
    bool _roverHBLost = false;
};

Arbitration::Arbitration(): Node("arbitration")
{
    for (size_t i = 0; i < _zeroCmd.enable.size(); ++i)
    {
        _zeroCmd.enable[i] = false;
        _zeroCmd.target_speed[i] = 0.0;
        _zeroCmd.current_speed[i] = 0.0;
        _zeroCmd.close_loop[i] = false;
    }

    _subBaseHr = this->create_subscription<std_msgs::msg::Empty>("/base/heartbeat",
                                                                 1,
                                                                 [this](const std_msgs::msg::Empty msg_)
                                                                 { this->cbHB(msg_, &_baseHBLost, _watchdogBase); });
    _subRoverHr = this->create_subscription<std_msgs::msg::Empty>("/rover/heartbeat",
                                                                  1,
                                                                  [this](const std_msgs::msg::Empty msg_)
                                                                  { this->cbHB(msg_, &_roverHBLost, _watchdogRover); });

    _subMotorCmdTeleop = this->create_subscription<rover_msgs::msg::PropulsionMotor>(
        "/rover/drive_train/cmd/in/teleop",
        1,
        std::bind(&Arbitration::cbPropulsionCmd, this, std::placeholders::_1));
    _subMotorCmdAuto = this->create_subscription<rover_msgs::msg::PropulsionMotor>(
        "/rover/drive_train/cmd/in/teleop",
        1,
        std::bind(&Arbitration::cbPropulsionCmd, this, std::placeholders::_1));

    _pubCmd = this->create_publisher<rover_msgs::msg::PropulsionMotor>("/rover/drive_train/cmd/out/motors", 1);
    _pubArbitrationStatus
        = this->create_publisher<rover_msgs::msg::DrivetrainArbitration>("/rover/drive_train/arbitration/status", 1);

    _srvControlDemux = this->create_service<rover_msgs::srv::DriveTrainArbitration>(
        "/rover/drive_train/set_arbitration",
        std::bind(&Arbitration::cbAbtr, this, std::placeholders::_1, std::placeholders::_2));

    _watchdogRover = this->create_wall_timer(std::chrono::milliseconds(500), [this]() { this->watchdog(&_roverHBLost); });
    _watchdogBase = this->create_wall_timer(std::chrono::milliseconds(500), [this]() { this->watchdog(&_baseHBLost); });

    _timerSendCmd = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&Arbitration::cbTimerSendCmd, this));
    _timerSendStatus = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&Arbitration::cbTimerSendStatus, this));
}

void Arbitration::cbTimerSendCmd()
{
    sendCmd();
}

void Arbitration::cbTimerSendStatus()
{
    _pubArbitrationStatus->publish(_arbitration);
}

void Arbitration::cbHB(const std_msgs::msg::Empty msg_, bool* _HBLostVar, rclcpp::TimerBase::SharedPtr _HBWatchdogTimer)
{
    REMOVE_UNUSED(msg_);

    *_HBLostVar = false;
    _HBWatchdogTimer->reset();
}

void Arbitration::cbPropulsionCmd(const rover_msgs::msg::PropulsionMotor msg_)
{
    _cmdTeleop = msg_;
}

void Arbitration::watchdog(bool* lostHB_)
{
    *lostHB_ = true;
}

void Arbitration::sendCmd()
{
    if (_baseHBLost || _roverHBLost)
    {
        _pubCmd->publish(_zeroCmd);
        return;
    }

    if (_arbitration.arbitration == rover_msgs::msg::DrivetrainArbitration::TELEOP)
    {
        _pubCmd->publish(_cmdTeleop);
    }
    else
    {
        _pubCmd->publish(_zeroCmd);
    }
}

void Arbitration::cbAbtr(const std::shared_ptr<rover_msgs::srv::DriveTrainArbitration::Request> request_,
                         std::shared_ptr<rover_msgs::srv::DriveTrainArbitration::Response> response_)
{
    if (request_->target_arbitration.arbitration == rover_msgs::msg::DrivetrainArbitration::NONE
        || request_->target_arbitration.arbitration == rover_msgs::msg::DrivetrainArbitration::TELEOP)
    {
        _arbitration = request_->target_arbitration;
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Can't see requested arbitration: %u, not implemented yet",
                     request_->target_arbitration.arbitration);
    }

    response_->current_arbitration = _arbitration;
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Arbitration>());
    rclcpp::shutdown();

    return 0;
}
