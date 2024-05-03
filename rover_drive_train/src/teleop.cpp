#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "rover_msgs/msg/joy.hpp"
#include "rover_msgs/msg/propulsion_motor.hpp"
#include "rover_msgs/msg/joy_demux_status.hpp"

//Class definition
class Teleop : public rclcpp::Node
{
    public:
        Teleop();

    private:
        //Private members
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

        float _controlMapFactor = 1.0f - _smallestRadius;
        
        //Private methods
        void getParams()
        {        
            this->declare_parameter("speedFactorCrawler", 0.01);
            this->declare_parameter("_speedFactorNormal", 0.26);
            this->declare_parameter("_speedFactorTurbo", 1.0);
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

        message.enable[rover_msgs::msg::PropulsionMotor::FRONT_LEFT] = true;
        message.enable[rover_msgs::msg::PropulsionMotor::FRONT_RIGHT] = true;
        message.enable[rover_msgs::msg::PropulsionMotor::REAR_LEFT] = true;
        message.enable[rover_msgs::msg::PropulsionMotor::REAR_RIGHT] = true;

        message.close_loop[rover_msgs::msg::PropulsionMotor::FRONT_LEFT] = false;
        message.close_loop[rover_msgs::msg::PropulsionMotor::FRONT_RIGHT] = false;
        message.close_loop[rover_msgs::msg::PropulsionMotor::REAR_LEFT] = false;
        message.close_loop[rover_msgs::msg::PropulsionMotor::REAR_RIGHT] = false;

        if(_deadmanSwitch)
        {
            float speedFactor = _speedFactorCrawler;

            if(_modeNormalEnable) 
            {
             speedFactor = _speedFactorNormal;
            } 
            if(_modeTurboEnable > 0.5 && _modeNormalEnable)
            {
             speedFactor = _speedFactorTurbo;
            }

            float speedLeftMotor = _linearInput * speedFactor;
            float speedRightMotor = _linearInput * speedFactor;

            if(_modeTankAngularInput != 0.0f) 
            {
                speedLeftMotor += _modeTankAngularInput * speedFactor;
                speedRightMotor -= _modeTankAngularInput * speedFactor;
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
        }
          
        _pub_teleop_in->publish(message);
    }
        
    rclcpp::Subscription<rover_msgs::msg::Joy>::SharedPtr _sub_joy_formated;
    rclcpp::Publisher<rover_msgs::msg::PropulsionMotor>::SharedPtr _pub_teleop_in;
};

//Constructor
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