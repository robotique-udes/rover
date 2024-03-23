#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "rover_msgs/msg/joy.hpp"
#include "rover_msgs/msg/motor_cmd.hpp"
#include "rover_msgs/msg/joy_demux_status.hpp"

using std::placeholders::_1;

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
            this->declare_parameter("_speedFactorNormal", 0.25);
            this->declare_parameter("_speedFactorTurbo", 1.0);
            this->declare_parameter("_smallestRadius", 0.30);

            this->get_parameter("speedFactorCrawler", _speedFactorCrawler);
            this->get_parameter("_speedFactorNormal", _speedFactorNormal);
            this->get_parameter("_speedFactorTurbo", _speedFactorTurbo);
            this->get_parameter("_smallestRadius", _smallestRadius);
        }

    void joyCallback(const rover_msgs::msg::Joy::SharedPtr msg)
    {
        rover_msgs::msg::MotorCmd message;

        _deadmanSwitch = msg->joy_data[rover_msgs::msg::Joy::L1];

        if(_deadmanSwitch)
        {
            _linearInput = msg->joy_data[rover_msgs::msg::Joy::JOYSTICK_LEFT_FRONT];
            _angularInput = msg->joy_data[rover_msgs::msg::Joy::JOYSTICK_LEFT_SIDE]; 
            _modeTankAngularInput = msg->joy_data[rover_msgs::msg::Joy::JOYSTICK_RIGHT_SIDE];
            _modeNormalEnable = msg->joy_data[rover_msgs::msg::Joy::R1];
            _modeTurboEnable = msg->joy_data[rover_msgs::msg::Joy::R2];

            float speed_factor = _speedFactorCrawler;

            if(_modeNormalEnable) 
            {
                speed_factor = _speedFactorNormal;
            } 
            if(_modeTurboEnable > 0.5 && _modeNormalEnable)
            {
                speed_factor = _speedFactorTurbo;
            }

            float f_speed_left_motor = _linearInput * speed_factor;
            float f_speed_right_motor = _linearInput * speed_factor;

            if(_modeTankAngularInput != 0.0f) 
            {
                f_speed_left_motor += _modeTankAngularInput * speed_factor;
                f_speed_right_motor -= _modeTankAngularInput * speed_factor;
            } 
            else 
            {
                float adjusted_factor;

                if (_angularInput > 0.0f) 
                {
                    adjusted_factor = 1.0f - _angularInput * _controlMapFactor;
                    f_speed_left_motor *= adjusted_factor < 0.01f ? 0.01f : adjusted_factor;
                } 
                else 
                {
                    adjusted_factor = 1.0f + _angularInput * _controlMapFactor;
                    f_speed_right_motor *= adjusted_factor < 0.01f ? 0.01f : adjusted_factor;
                }
            }

            message.front_left = f_speed_left_motor;
            message.rear_left = f_speed_left_motor;
            message.front_right = f_speed_right_motor;
            message.rear_right = f_speed_right_motor;
        }

        _pub_teleop_in->publish(message);
    }
        
        rclcpp::Subscription<rover_msgs::msg::Joy>::SharedPtr _sub_joy_formated;
        rclcpp::Publisher<rover_msgs::msg::MotorCmd>::SharedPtr _pub_teleop_in;




};

//Constructor
Teleop::Teleop() : Node("teleop")
{
    getParams();

    _sub_joy_formated = this->create_subscription<rover_msgs::msg::Joy>("/rover/drive_train/joy",
                                                                         1,
                                                                         std::bind(&Teleop::joyCallback, this, std::placeholders::_1));

    _pub_teleop_in = this->create_publisher<rover_msgs::msg::MotorCmd>("/rover/drive_train/cmd/in/teleop", 1);
                                                                   
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Teleop>());
    rclcpp::shutdown();
    
    return 0;
}