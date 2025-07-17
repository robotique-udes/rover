#include <rclcpp/rclcpp.hpp>
#include <rover_can2/rover_can2.hpp>
#include <rover_can2/msgs/arm_speed_cmd.hpp>
#include <rover_can2/msgs/arm_position_status.hpp>

class ArmJointSimulator : public RoverCan2::Device<RoverCan2::Publisher<RoverCan2::Msgs::ArmSpeedCmd>,
                                                   RoverCan2::SubscriberMember<RoverCan2::Msgs::ArmPositionStatus, ArmJoint>>,
                          rclcpp::Node
{
  private:
    static constexpr double ARM_POSITION_STATUS_PUBLISH_FREQUENCY_HZ = 50;
    static constexpr uint32_t CAN_PUBLISH_PERIOD_MS
        = static_cast<uint32_t>(ROUND(1'000.0F / ARM_POSITION_STATUS_PUBLISH_FREQUENCY_HZ));

  public:
    ArmJointSimulator():
        Node("arm_joint_simulator")
    {
        // Set up CAN device ID
        _deviceID = RoverCan2::Constant::eDeviceId::JL_CONTROLLER;

        // Print info
        RCLCPP_INFO(get_logger(), "Starting arm joint simulator for device ID: 0x%x", static_cast<int>(_deviceID));

        // Initialize CAN driver and set up messaging
        std::shared_ptr<RoverCan2::Drivers::DriverLinux> _can_driver = std::make_shared<RoverCan2::Drivers::DriverLinux>("vcan0");

        // Initialize the driver
        _can_driver->init();

        // Create a manager to handle CAN communication
        _can_manager = std::make_shared<RoverCan2::ManagerMaster<RoverCan2::Drivers::DriverLinux>>(
            *_can_driver,
            [](RoverCan2::Constant::eDeviceId, const RoverCan2::Msgs::ErrorState&)
            {
                // Error state handler (can be empty for simulator)
            });

        // Create CAN publisher for position status
        _position_pub = std::make_shared<RoverCan2::Publisher<RoverCan2::Msgs::ArmPositionStatus>>(_deviceID);

        // Create CAN subscriber for speed commands
        _can_manager->registerSubscriber<RoverCan2::Msgs::ArmSpeedCmd>(_deviceID,
                                                                       [this](const RoverCan2::Msgs::ArmSpeedCmd& msg)
                                                                       {
                                                                           this->handle_speed_cmd(msg);
                                                                       });

        // Create timer to simulate joint physics and publish status
        _update_timer = this->create_wall_timer(std::chrono::milliseconds(CAN_PUBLISH_PERIOD_MS),
                                                std::bind(&ArmJointSimulator::update_and_publish, this));

        // Initialize joint state
        _current_position = 0.0;
        _current_speed = 0.0;
        _target_speed = 0.0;
        _min_position = 0;
        _max_position = 1;

        RCLCPP_INFO(get_logger(), "Arm joint simulator initialized");
    }

  private:
    void handle_speed_cmd(const RoverCan2::Msgs::ArmSpeedCmd& msg)
    {
        _target_speed = msg.getData().targetSpeed;
        RCLCPP_INFO(get_logger(), "Received speed command: %.2f", _target_speed);
    }

    void update_and_publish()
    {
        // Calculate time step
        auto now = this->now();
        double dt = 0.02;  // Default to 20ms

        if (_last_update_time.nanoseconds() > 0)
        {
            dt = (now - _last_update_time).seconds();
        }
        _last_update_time = now;

        // Simple physics simulation
        // Speed approaches target with a simple low-pass filter
        _current_speed = _current_speed * 0.9 + _target_speed * 0.1;

        // Update position based on current speed
        _current_position += _current_speed * dt;

        // Apply position limits
        if (_current_position < _min_position)
        {
            _current_position = _min_position;
            _current_speed = 0.0;
        }
        else if (_current_position > _max_position)
        {
            _current_position = _max_position;
            _current_speed = 0.0;
        }

        // Create and publish status message
        RoverCan2::Msgs::ArmPositionStatus status_msg;
        status_msg.data().position = static_cast<float>(_current_position);
        status_msg.data().speed = static_cast<float>(_current_speed);
        _position_pub->sendMsg(status_msg);
        _can_manager->update();
        RCLCPP_DEBUG(get_logger(), "Published position: %.2f, speed: %.2f", _current_position, _current_speed);
    }

    // CAN communication
    RoverCan2::Constant::eDeviceId _deviceID;
    std::shared_ptr<RoverCan2::Publisher<RoverCan2::Msgs::ArmPositionStatus>> _position_pub;
    std::shared_ptr<RoverCan2::Subscriber<RoverCan2::Msgs::ArmSpeedCmd>> _speed_cmd_sub;
    std::shared_ptr<RoverCan2::Drivers::DriverLinux> _can_driver;
    std::shared_ptr<RoverCan2::ManagerMaster<RoverCan2::Drivers::DriverLinux>> _can_manager;

    // Timer
    rclcpp::TimerBase::SharedPtr _update_timer;
    rclcpp::Time _last_update_time;

    // Joint state
    double _current_position;
    double _current_speed;
    double _target_speed;
    double _min_position;
    double _max_position;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmJointSimulator>());
    rclcpp::shutdown();
    return 0;
}