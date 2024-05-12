#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "rover_msgs/msg/joy.hpp"
#include "rover_msgs/msg/propulsion_motor.hpp"
#include "rover_msgs/msg/joy_demux_status.hpp"

// Class definition
class Teleop : public rclcpp::Node
{
public:
    Teleop();

private:
    // Private members
    float _speedFactorCrawler;
    float _speedFactorNormal;
    float _speedFactorTurbo;
    float _smallestRadius;

    float _deadmanSwitch;

    float _linearInput;
    float _angularInput;

    float _modeTankAngularInput;
    float _modeNormalEnable;
    float _modeTurboEnable;

    float _bumbaFast;
    float _bumbaBurn;
    float _bumbaDrift;

    float _controlMapFactor = 1.0f - _smallestRadius;

    // Private methods
    void getParams()
    {
        this->declare_parameter("speedFactorCrawler", 0.03);
        this->declare_parameter("_speedFactorNormal", 0.25);
        this->declare_parameter("_speedFactorTurbo", 0.5);
        this->declare_parameter("_smallestRadius", 0.30);

        this->get_parameter("speedFactorCrawler", _speedFactorCrawler);
        this->get_parameter("_speedFactorNormal", _speedFactorNormal);
        this->get_parameter("_speedFactorTurbo", _speedFactorTurbo);
        this->get_parameter("_smallestRadius", _smallestRadius);
    }

    void joyCallback(const rover_msgs::msg::Joy::SharedPtr msg)
    {
        rover_msgs::msg::PropulsionMotor message;

        _deadmanSwitch = msg->joy_data[rover_msgs::msg::Joy::L1];

        _linearInput = msg->joy_data[rover_msgs::msg::Joy::JOYSTICK_LEFT_FRONT];
        _angularInput = msg->joy_data[rover_msgs::msg::Joy::JOYSTICK_LEFT_SIDE];
        _modeTankAngularInput = msg->joy_data[rover_msgs::msg::Joy::JOYSTICK_RIGHT_SIDE];
        _modeNormalEnable = msg->joy_data[rover_msgs::msg::Joy::R1];
        _modeTurboEnable = msg->joy_data[rover_msgs::msg::Joy::R2];
        _bumbaFast = msg->joy_data[rover_msgs::msg::Joy::A];
        _bumbaBurn = msg->joy_data[rover_msgs::msg::Joy::X];
        _bumbaDrift = msg->joy_data[rover_msgs::msg::Joy::B];

        message.enable[rover_msgs::msg::PropulsionMotor::FRONT_LEFT] = true;
        message.enable[rover_msgs::msg::PropulsionMotor::FRONT_RIGHT] = true;
        message.enable[rover_msgs::msg::PropulsionMotor::REAR_LEFT] = true;
        message.enable[rover_msgs::msg::PropulsionMotor::REAR_RIGHT] = true;

        message.close_loop[rover_msgs::msg::PropulsionMotor::FRONT_LEFT] = false;
        message.close_loop[rover_msgs::msg::PropulsionMotor::FRONT_RIGHT] = false;
        message.close_loop[rover_msgs::msg::PropulsionMotor::REAR_LEFT] = false;
        message.close_loop[rover_msgs::msg::PropulsionMotor::REAR_RIGHT] = false;

        if (_deadmanSwitch)
        {
            float speedFactor = _speedFactorCrawler;

            if (_modeNormalEnable)
            {
                speedFactor = _speedFactorNormal;
            }
            if (_modeTurboEnable > 0.5 && _modeNormalEnable)
            {
                speedFactor = _speedFactorTurbo;
            }
            if (_bumbaFast)
            {
                speedFactor = 1.0;
            }

            float speedLeftMotor = _linearInput * speedFactor;
            float speedRightMotor = _linearInput * speedFactor;

            if (_modeTankAngularInput != 0.0f)
            {
                speedLeftMotor += -1.0f * _modeTankAngularInput * speedFactor;
                speedRightMotor -= -1.0f * _modeTankAngularInput * speedFactor;
            }

            else
            {
                float adjusted_factor;

                if (_angularInput > 0.0f)
                {
                    adjusted_factor = 1.0f - _angularInput * _controlMapFactor;
                    speedLeftMotor *= adjusted_factor < 0.01f ? 0.01f : adjusted_factor;
                }
                else
                {
                    adjusted_factor = 1.0f + _angularInput * _controlMapFactor;
                    speedRightMotor *= adjusted_factor < 0.01f ? 0.01f : adjusted_factor;
                }
                
            }
            
            message.target_speed[rover_msgs::msg::PropulsionMotor::FRONT_LEFT] = speedLeftMotor;
            message.target_speed[rover_msgs::msg::PropulsionMotor::FRONT_RIGHT] = speedRightMotor;
            message.target_speed[rover_msgs::msg::PropulsionMotor::REAR_LEFT] = speedLeftMotor;
            message.target_speed[rover_msgs::msg::PropulsionMotor::REAR_RIGHT] = speedRightMotor;
            
            if (_bumbaBurn > 0.5 || _bumbaDrift >  0.5) //BUMBAAACLOAT
            {
                if(_bumbaBurn > 0.5)
                {
                    message.target_speed[rover_msgs::msg::PropulsionMotor::FRONT_LEFT] = 1.0f;
                    message.target_speed[rover_msgs::msg::PropulsionMotor::FRONT_RIGHT] = 1.0f;
                    message.target_speed[rover_msgs::msg::PropulsionMotor::REAR_LEFT] = -1.0f;
                    message.target_speed[rover_msgs::msg::PropulsionMotor::REAR_RIGHT] = -1.0f;

                }
                else if(_bumbaDrift > 0.5)
                {
                    message.target_speed[rover_msgs::msg::PropulsionMotor::FRONT_LEFT] = -0.4f;
                    message.target_speed[rover_msgs::msg::PropulsionMotor::FRONT_RIGHT] = -0.8f;
                    message.target_speed[rover_msgs::msg::PropulsionMotor::REAR_LEFT] = 1.0f;
                    message.target_speed[rover_msgs::msg::PropulsionMotor::REAR_RIGHT] = 1.0f;
                }
            }
        }
        
        _pub_teleop_in->publish(message);
    }

    rclcpp::Subscription<rover_msgs::msg::Joy>::SharedPtr _sub_joy_formated;
    rclcpp::Publisher<rover_msgs::msg::PropulsionMotor>::SharedPtr _pub_teleop_in;
};

// Constructor
Teleop::Teleop() : Node("teleop")
{
    this->getParams();

    _sub_joy_formated = this->create_subscription<rover_msgs::msg::Joy>("/rover/drive_train/joy",
                                                                        1,
                                                                        std::bind(&Teleop::joyCallback, this, std::placeholders::_1));

    _pub_teleop_in = this->create_publisher<rover_msgs::msg::PropulsionMotor>("/rover/drive_train/cmd/in/teleop", 1);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Teleop>());
    rclcpp::shutdown();

    return 0;
}
