#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/joy.hpp"
#include "rover_msgs/msg/joy_demux_status.hpp"
#include "rover_msgs/msg/propulsion_motor.hpp"
#include "std_msgs/msg/empty.hpp"
#include <rover_lib2/helpers/constants.hpp>

// Class definition
class Teleop : public rclcpp::Node
{
    static constexpr const char* TOPIC_JOY = "/base/joy/drive_train";
    static constexpr const char* TOPIC_WHEEL_CMD = "/rover/drive_train/wheels_cmd_telelop";

  public:
    Teleop();

  private:
    bool floatToBool(float variable)
    {
        return variable == 1.0f;
    }

    void CB_joy(const rover_msgs::msg::Joy& msg)
    {
        rover_msgs::msg::PropulsionMotor message;

        float deadmanSwitch = msg.joy_data[Constants::DriveTrain::KeyBinding::DEADMAN_SWITCH];
        float linearInput = msg.joy_data[Constants::DriveTrain::KeyBinding::LINEAR_INPUT];
        float angularInput = msg.joy_data[Constants::DriveTrain::KeyBinding::ANGULAR_INPUT];
        float modeTankAngularInput = msg.joy_data[Constants::DriveTrain::KeyBinding::MODE_TANK_ANGULAR_INPUT];
        float modeNormalEnable = msg.joy_data[Constants::DriveTrain::KeyBinding::MODE_NORMAL_ENABLE];
        float modeTurboEnable = msg.joy_data[Constants::DriveTrain::KeyBinding::MODE_TURBO_ENABLE];

        if (floatToBool(deadmanSwitch))
        {
            float speedFactor = Constants::DriveTrain::SPEED_FACTOR_CRAWLER;

            if (floatToBool(modeNormalEnable))
            {
                speedFactor = Constants::DriveTrain::SPEED_FACTOR_NORMAL;
            }
            if (modeTurboEnable > 0.5f && floatToBool(modeNormalEnable))
            {
                speedFactor = Constants::DriveTrain::SPEED_FACTOR_TURBO;
            }

            float speedLeftMotor = linearInput * speedFactor;
            float speedRightMotor = linearInput * speedFactor;

            if (modeTankAngularInput != 0.0f)
            {
                speedLeftMotor += -1.0f * modeTankAngularInput * speedFactor;
                speedRightMotor -= -1.0f * modeTankAngularInput * speedFactor;
            }

            else
            {
                float controlMapFactor = 1.0f - Constants::DriveTrain::SMALLEST_RADIUS;
                float adjustedFactor;

                if (angularInput > 0.0f)
                {
                    adjustedFactor = 1.0f - angularInput * controlMapFactor;
                    speedLeftMotor *= adjustedFactor < 0.01f ? 0.01f : adjustedFactor;
                }
                else
                {
                    adjustedFactor = 1.0f + angularInput * controlMapFactor;
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
                                                                        std::bind(&Teleop::CB_joy, this, std::placeholders::_1));

    _pub_teleop_in = this->create_publisher<rover_msgs::msg::PropulsionMotor>(TOPIC_WHEEL_CMD, QOS_DEFAULT);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Teleop>());
    rclcpp::shutdown();

    return 0;
}
