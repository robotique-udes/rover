#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription_options.hpp>
#include <rover_msgs/msg/joy.hpp>
#include <rover_msgs/msg/joy_demux_status.hpp>
#include <rover_msgs/msg/propulsion_motor.hpp>
#include <std_msgs/msg/empty.hpp>
#include <rover_lib2/helpers/constants.hpp>
#include <rover_lib2/helpers/macros.hpp>

// Class definition
class Teleop : public rclcpp::Node
{
    static constexpr const char* TOPIC_JOY = "/base/joy/drive_train";
    static constexpr const char* TOPIC_WHEEL_CMD = "/rover/drive_train/wheels_cmd_telelop";
    static constexpr float CAR_CONTROL_MAP_FACTOR = 1.0f - Constants::DriveTrain::SMALLEST_RADIUS;
    static constexpr float CAR_MODE_INPUT_BYPASS_THREASHOLD = 0.05F;
    static constexpr float CAR_MODE_TURN_DEADZONE = 0.50F / 2.0F;  // 50% total, 50%/2 right + 50%/2 left
    static constexpr std::chrono::milliseconds TELEOP_DEADLINE = std::chrono::milliseconds(200);
    static constexpr std::chrono::milliseconds TELEOP_LEASE_DURATION = std::chrono::milliseconds(300);

  public:
    Teleop();

  private:
    bool floatToBool(float variable_) const
    {
        return variable_ == 1.0f;
    }

    void CB_joy(const rover_msgs::msg::Joy& msg_) const
    {
        rover_msgs::msg::PropulsionMotor message;

        float deadmanSwitch = msg_.joy_data[std::to_underlying(Constants::Keybinds::DriveTrain::DEADMAN_SWITCH)];
        float linearInput = msg_.joy_data[std::to_underlying(Constants::Keybinds::DriveTrain::LINEAR_INPUT)];
        float angularInput = msg_.joy_data[std::to_underlying(Constants::Keybinds::DriveTrain::ANGULAR_INPUT)];
        float modeTankAngularInput = msg_.joy_data[std::to_underlying(Constants::Keybinds::DriveTrain::MODE_TANK_ANGULAR_INPUT)];
        float modeNormalEnable = msg_.joy_data[std::to_underlying(Constants::Keybinds::DriveTrain::MODE_NORMAL_ENABLE)];
        float modeTurboEnable = msg_.joy_data[std::to_underlying(Constants::Keybinds::DriveTrain::MODE_TURBO_ENABLE)];

        if (this->floatToBool(deadmanSwitch))
        {
            float speedFactor = Constants::DriveTrain::SPEED_FACTOR_CRAWLER;

            if (modeTurboEnable > 0.5f && this->floatToBool(modeNormalEnable))
            {
                speedFactor = Constants::DriveTrain::SPEED_FACTOR_TURBO;
            }
            else if (this->floatToBool(modeNormalEnable))
            {
                speedFactor = Constants::DriveTrain::SPEED_FACTOR_NORMAL;
            }

            float speedLeftMotor = linearInput * speedFactor;
            float speedRightMotor = linearInput * speedFactor;

            if (!IN_ERROR(modeTankAngularInput, CAR_MODE_INPUT_BYPASS_THREASHOLD, 0.0F))
            {
                speedLeftMotor += -1.0f * modeTankAngularInput * speedFactor;
                speedRightMotor -= -1.0f * modeTankAngularInput * speedFactor;
            }
            else if (!IN_ERROR(angularInput, CAR_MODE_TURN_DEADZONE, 0.0F))
            {
                float adjustedFactor = 0.0F;

                if (angularInput > 0.0f)
                {
                    angularInput = MAP(angularInput, CAR_MODE_TURN_DEADZONE, 1.0F, 0.0F, 1.0F);
                    adjustedFactor = 1.0f - angularInput * CAR_CONTROL_MAP_FACTOR;
                    speedLeftMotor *= adjustedFactor < 0.01f ? 0.01f : adjustedFactor;
                }
                else
                {
                    angularInput = MAP(angularInput, -1.0F, -CAR_MODE_TURN_DEADZONE, -1.0F, 0.0F);
                    adjustedFactor = 1.0f + angularInput * CAR_CONTROL_MAP_FACTOR;
                    speedRightMotor *= adjustedFactor < 0.01f ? 0.01f : adjustedFactor;
                }
            }

            message.target_speed[rover_msgs::msg::PropulsionMotor::MOTOR_FRONT_LEFT] = speedLeftMotor;
            message.target_speed[rover_msgs::msg::PropulsionMotor::MOTOR_FRONT_RIGHT] = speedRightMotor;
            message.target_speed[rover_msgs::msg::PropulsionMotor::MOTOR_REAR_LEFT] = speedLeftMotor;
            message.target_speed[rover_msgs::msg::PropulsionMotor::MOTOR_REAR_RIGHT] = speedRightMotor;
        }

        _pub_teleop_in->publish(message);
    }

    rclcpp::Subscription<rover_msgs::msg::Joy>::SharedPtr _sub_joy_formated;
    rclcpp::Publisher<rover_msgs::msg::PropulsionMotor>::SharedPtr _pub_teleop_in;
};

// Constructor
Teleop::Teleop():
    Node("teleop")
{
    _sub_joy_formated = this->create_subscription<rover_msgs::msg::Joy>(TOPIC_JOY,
                                                                        QOS_DEFAULT,
                                                                        [this](const rover_msgs::msg::Joy& msg_)
                                                                        {
                                                                            this->CB_joy(msg_);
                                                                        });

    rclcpp::QoS teleopQos(rclcpp::KeepLast(1));
    teleopQos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    teleopQos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    teleopQos.deadline(TELEOP_DEADLINE);
    teleopQos.liveliness(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC);
    teleopQos.liveliness_lease_duration(TELEOP_LEASE_DURATION);
    _pub_teleop_in = this->create_publisher<rover_msgs::msg::PropulsionMotor>(TOPIC_WHEEL_CMD, teleopQos);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Teleop>());
    rclcpp::shutdown();

    return 0;
}
