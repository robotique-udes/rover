#include <rover_lib2/helpers/macros.hpp>
#include <rover_lib2/helpers/constants.hpp>

#include <rover_msgs/srv/drive_train_arbitration.hpp>
#include <rover_msgs/msg/drivetrain_arbitration.hpp>
#include <rover_msgs/msg/joy.hpp>
#include <rover_msgs/msg/joy_demux_status.hpp>
#include <rover_msgs/msg/propulsion_motor.hpp>
#include <std_msgs/msg/empty.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription_options.hpp>

class Arbitration : public rclcpp::Node
{
    static constexpr const char* TOPIC_HEARTBEAT_BASE = "/base/heartbeat";
    static constexpr const char* TOPIC_HEARTBEAT_ROVER = "/rover/heartbeat";
    static constexpr const char* TOPIC_CMD_WHEELS_AUTO = "/rover/drive_train/wheels_cmd_auto";
    static constexpr const char* TOPIC_CMD_WHEELS_TELEOP = "/rover/drive_train/wheels_cmd_telelop";
    static constexpr const char* TOPIC_CMD_WHEELS_OUT = "/rover/drive_train/wheels_cmd_out";
    static constexpr const char* SERVICE_ARBITRATION_CONTROL = "/rover/drive_train/demux_control";
    static constexpr const char* TOPIC_ARBITRATION_STATUS = "/rover/drive_train/demux_status";
    static constexpr std::chrono::milliseconds TELEOP_DEADLINE = std::chrono::milliseconds(200);
    static constexpr std::chrono::milliseconds TELEOP_LEASE_DURATION = std::chrono::milliseconds(300);

  public:
    Arbitration();

  private:
    void cbTimerSendCmd(void) const;
    void cbTimerSendStatus(void) const;
    void watchdog(bool* lostHB_) const;
    void cbPropulsionCmd(const rover_msgs::msg::PropulsionMotor& msg_);
    void cbHB(const std_msgs::msg::Empty msg_, bool* HBLostVar_, rclcpp::TimerBase::SharedPtr HBWatchdogTimer_) const;
    void cbAbtr(const std::shared_ptr<rover_msgs::srv::DriveTrainArbitration::Request> request_,
                std::shared_ptr<rover_msgs::srv::DriveTrainArbitration::Response> response_);
    void sendCmd(void) const;

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
    bool _deadlineWarningActive = false;
};

Arbitration::Arbitration():
    Node("arbitration")
{
    _arbitration.arbitration = rover_msgs::msg::DrivetrainArbitration::TELEOP;

    for (size_t i = 0; i < rover_msgs::msg::PropulsionMotor::MOTOR_MAX; ++i)
    {
        _zeroCmd.target_speed[i] = 0.0;
        _zeroCmd.current_speed[i] = 0.0;
    }
    _cmdTeleop = _zeroCmd;

    _subBaseHr = this->create_subscription<std_msgs::msg::Empty>(TOPIC_HEARTBEAT_BASE,
                                                                 QOS_DEFAULT,
                                                                 [this](const std_msgs::msg::Empty msg_)
                                                                 {
                                                                     this->cbHB(msg_, &_baseHBLost, _watchdogBase);
                                                                 });
    _subRoverHr = this->create_subscription<std_msgs::msg::Empty>(TOPIC_HEARTBEAT_ROVER,
                                                                  QOS_DEFAULT,
                                                                  [this](const std_msgs::msg::Empty msg_)
                                                                  {
                                                                      this->cbHB(msg_, &_roverHBLost, _watchdogRover);
                                                                  });
    rclcpp::QoS teleopQos(rclcpp::KeepLast(1));
    teleopQos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    teleopQos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    teleopQos.deadline(TELEOP_DEADLINE);
    teleopQos.liveliness(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC);
    teleopQos.liveliness_lease_duration(TELEOP_LEASE_DURATION);

    rclcpp::SubscriptionOptions teleopSubOptions;
    teleopSubOptions.event_callbacks.deadline_callback = [this](rclcpp::QOSDeadlineRequestedInfo& info_)
    {
        if (info_.total_count_change > 0 && !_deadlineWarningActive)
        {
            _deadlineWarningActive = true;
            RCLCPP_WARN(this->get_logger(),
                        "Teleop deadline missed: total=%d change=%d",
                        info_.total_count,
                        info_.total_count_change);
            _cmdTeleop = _zeroCmd;
        }
    };

    teleopSubOptions.event_callbacks.liveliness_callback = [this](rclcpp::QOSLivelinessChangedInfo& info_)
    {
        if (info_.not_alive_count_change > 0)
        {
            RCLCPP_WARN(this->get_logger(),
                        "Teleop liveliness lost: alive=%d not_alive=%d",
                        info_.alive_count,
                        info_.not_alive_count);
            _cmdTeleop = _zeroCmd;
        }
    };

    _subMotorCmdTeleop = this->create_subscription<rover_msgs::msg::PropulsionMotor>(
        TOPIC_CMD_WHEELS_TELEOP,
        teleopQos,
        [this](const rover_msgs::msg::PropulsionMotor& msg_)
        {
            this->cbPropulsionCmd(msg_);
        },
        teleopSubOptions);
    _subMotorCmdAuto
        = this->create_subscription<rover_msgs::msg::PropulsionMotor>(TOPIC_CMD_WHEELS_AUTO,
                                                                      QOS_DEFAULT,
                                                                      [this](const rover_msgs::msg::PropulsionMotor& msg_)
                                                                      {
                                                                          this->cbPropulsionCmd(msg_);
                                                                      });

    _pubCmd = this->create_publisher<rover_msgs::msg::PropulsionMotor>(TOPIC_CMD_WHEELS_OUT, QOS_DEFAULT);
    _pubArbitrationStatus = this->create_publisher<rover_msgs::msg::DrivetrainArbitration>(TOPIC_ARBITRATION_STATUS, QOS_DEFAULT);

    _srvControlDemux = this->create_service<rover_msgs::srv::DriveTrainArbitration>(
        SERVICE_ARBITRATION_CONTROL,
        [this](const std::shared_ptr<rover_msgs::srv::DriveTrainArbitration::Request> request_,
               std::shared_ptr<rover_msgs::srv::DriveTrainArbitration::Response> response_)
        {
            this->cbAbtr(request_, response_);
        });

    _watchdogRover = this->create_wall_timer(std::chrono::milliseconds(500),
                                             [this]()
                                             {
                                                 this->watchdog(&_roverHBLost);
                                             });
    _watchdogBase = this->create_wall_timer(std::chrono::milliseconds(500),
                                            [this]()
                                            {
                                                this->watchdog(&_baseHBLost);
                                            });

    _timerSendCmd = this->create_wall_timer(std::chrono::milliseconds(10),
                                            [this]()
                                            {
                                                this->cbTimerSendCmd();
                                            });
    _timerSendStatus = this->create_wall_timer(std::chrono::milliseconds(1000),
                                               [this]()
                                               {
                                                   this->cbTimerSendStatus();
                                               });
}

void Arbitration::cbTimerSendCmd(void) const
{
    this->sendCmd();
}

void Arbitration::cbTimerSendStatus(void) const
{
    _pubArbitrationStatus->publish(_arbitration);
}

void Arbitration::cbHB(const std_msgs::msg::Empty /*msg_*/, bool* HBLostVar_, rclcpp::TimerBase::SharedPtr HBWatchdogTimer_) const
{
    *HBLostVar_ = false;
    HBWatchdogTimer_->reset();
}

void Arbitration::cbPropulsionCmd(const rover_msgs::msg::PropulsionMotor& msg_)
{
    _deadlineWarningActive = false;
    _cmdTeleop = msg_;
}

void Arbitration::watchdog(bool* lostHB_) const
{
    *lostHB_ = true;
}

void Arbitration::sendCmd() const
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
