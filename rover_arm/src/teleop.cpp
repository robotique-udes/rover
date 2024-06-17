#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "rover_msgs/msg/joy.hpp"
#include "rover_msgs/msg/propulsion_motor.hpp"
#include "rover_msgs/msg/joy_demux_status.hpp"
#include "rover_msgs/msg/arm_cmd.hpp"

class Teleop : public rclcpp::Node
{
    public:
        Teleop();

    private:
        Teleop* teleop_test;
        
        //Private members
        // =========================================================================
        float _speedFactorCrawler;
        float _speedFactorNormal;
        float _speedFactorTurbo;

        float _linkj0Toj1 = 529.5715;
        float _linkj1ToJ2 = 608.45785;
        float _endEffectorLength = 210.1945;

        float _linearControl;
        float _j0Control;
        float _j1Control;
        float _j2Control;
        float _gripperUD;
        float _gripperLR;
        float _gripperO;
        float _gripperC;

        float _deadmanSwitch;
        float _gripperMode;

        float _forwardKinematic = 1;
        float _inverseKinematic;

        float _modeNormalEnable;
        float _modeTurboEnable;        

        //Private methods
        // =========================================================================
        void getParams()
        {
            this->declare_parameter("speedFactorCrawler", 0.03);
            this->declare_parameter("speedFactorNormal", 0.25);
            this->declare_parameter("speedFactorTurbo", 0.3);

            this->get_parameter("speedFactorCrawler", _speedFactorCrawler);
            this->get_parameter("speedFactorNormal", _speedFactorNormal);
            this->get_parameter("speedFactorTurbo", _speedFactorTurbo);
        }

        void joyCallback(const rover_msgs::msg::Joy::SharedPtr joyMsg)
        {
            rover_msgs::msg::ArmCmd armMsg;

            //Mode selectiion
            // =========================================================================            
            _deadmanSwitch = joyMsg->joy_data[rover_msgs::msg::Joy::L1];
            _gripperMode = joyMsg->joy_data[rover_msgs::msg::Joy::R1];

            //Arm controls
            // =========================================================================            
            _linearControl = joyMsg->joy_data[rover_msgs::msg::Joy::JOYSTICK_LEFT_SIDE];
            _j0Control = joyMsg->joy_data[rover_msgs::msg::Joy::JOYSTICK_RIGHT_SIDE];
            _j1Control = joyMsg->joy_data[rover_msgs::msg::Joy::JOYSTICK_LEFT_FRONT];
            _j2Control = joyMsg->joy_data[rover_msgs::msg::Joy::JOYSTICK_RIGHT_FRONT];
            
            //Gripper controls
            // =========================================================================            
            _gripperUD = joyMsg->joy_data[rover_msgs::msg::Joy::JOYSTICK_LEFT_FRONT];
            _gripperLR = joyMsg->joy_data[rover_msgs::msg::Joy::JOYSTICK_RIGHT_SIDE];
            _gripperO = joyMsg->joy_data[rover_msgs::msg::Joy::R2];
            _gripperC = joyMsg->joy_data[rover_msgs::msg::Joy::L2];

            //Msg control
            // =========================================================================            
            armMsg.enable[rover_msgs::msg::ArmCmd::LINEAR] = true;
            armMsg.enable[rover_msgs::msg::ArmCmd::J0] = true;
            armMsg.enable[rover_msgs::msg::ArmCmd::J1] = true;
            armMsg.enable[rover_msgs::msg::ArmCmd::J2] = true;
            armMsg.enable[rover_msgs::msg::ArmCmd::GRIPPERLR] = true;
            armMsg.enable[rover_msgs::msg::ArmCmd::GRIPPERUD] = true;
            armMsg.enable[rover_msgs::msg::ArmCmd::GRIPPERO] = true;
            armMsg.enable[rover_msgs::msg::ArmCmd::GRIPPERC] = true;

            armMsg.close_loop[rover_msgs::msg::ArmCmd::LINEAR] = false;
            armMsg.close_loop[rover_msgs::msg::ArmCmd::J0] = false;
            armMsg.close_loop[rover_msgs::msg::ArmCmd::J1] = false;
            armMsg.close_loop[rover_msgs::msg::ArmCmd::J2] = false;
            armMsg.close_loop[rover_msgs::msg::ArmCmd::GRIPPERLR] = false;
            armMsg.close_loop[rover_msgs::msg::ArmCmd::GRIPPERUD] = false;
            armMsg.close_loop[rover_msgs::msg::ArmCmd::GRIPPERO] = false;
            armMsg.close_loop[rover_msgs::msg::ArmCmd::GRIPPERC] = true;

            //Control logic
            // =========================================================================
            if(_deadmanSwitch)
            {
                float speedFactor = _speedFactorCrawler;

                if(_modeNormalEnable)
                {
                    speedFactor = _speedFactorNormal;
                }
                else if(_modeTurboEnable)
                {
                    speedFactor = _speedFactorTurbo;
                }

                if(_forwardKinematic)
                {
                    if(_gripperMode)
                    {
                        armMsg.target_position[rover_msgs::msg::ArmCmd::GRIPPERLR] += _gripperLR * speedFactor;
                        armMsg.target_position[rover_msgs::msg::ArmCmd::GRIPPERUD] += _gripperUD * speedFactor;
                        armMsg.target_position[rover_msgs::msg::ArmCmd::GRIPPERO] += _gripperO * speedFactor;
                        armMsg.target_position[rover_msgs::msg::ArmCmd::GRIPPERC] += _gripperC * speedFactor;
                    }
                    else
                    {
                        armMsg.target_position[rover_msgs::msg::ArmCmd::LINEAR] += _linearControl * speedFactor;
                        armMsg.target_position[rover_msgs::msg::ArmCmd::J0] += _j0Control * speedFactor;
                        armMsg.target_position[rover_msgs::msg::ArmCmd::J1] += _j1Control * speedFactor;
                        armMsg.target_position[rover_msgs::msg::ArmCmd::J2] += _j2Control * speedFactor;
                    }
                }
                else if(_inverseKinematic)
                {
                    RCLCPP_INFO(this->get_logger(), "Inverse kinematics mode NOT YET AVAILABLE");
                }             
            }
            else
            {
                armMsg.target_position[rover_msgs::msg::ArmCmd::LINEAR] = 0.0f;
                armMsg.target_position[rover_msgs::msg::ArmCmd::J0] = 0.0f;
                armMsg.target_position[rover_msgs::msg::ArmCmd::J1] = 0.0f;
                armMsg.target_position[rover_msgs::msg::ArmCmd::J2] = 0.0f;
                armMsg.target_position[rover_msgs::msg::ArmCmd::GRIPPERLR] = 0.0f;
                armMsg.target_position[rover_msgs::msg::ArmCmd::GRIPPERUD] = 0.0f;
                armMsg.target_position[rover_msgs::msg::ArmCmd::GRIPPERO] = 0.0f;
                armMsg.target_position[rover_msgs::msg::ArmCmd::GRIPPERC] = 0.0f;
            }

            _pub_arm_teleop_in->publish(armMsg);

        }

    rclcpp::Subscription<rover_msgs::msg::Joy>::SharedPtr _sub_joy_arm;
    rclcpp::Publisher<rover_msgs::msg::ArmCmd>::SharedPtr _pub_arm_teleop_in;
};

// Teleop class constructor
// =========================================================================   
Teleop::Teleop() : Node("teleop")
{
    this->getParams();

    _sub_joy_arm = this->create_subscription<rover_msgs::msg::Joy>("/rover/drive_train/joy",
                                                                    1,
                                                                    std::bind(&Teleop::joyCallback, this, std::placeholders::_1));
    _pub_arm_teleop_in = this->create_publisher<rover_msgs::msg::ArmCmd>("rover/arm/cmd/in/teleop", 1);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Teleop>());
    rclcpp::shutdown();
}