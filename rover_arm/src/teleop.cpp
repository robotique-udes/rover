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
        ~Teleop();

    private:
        //Private members
        // =========================================================================
        float _speedFactorCrawler;
        float _speedFactorNormal;
        float _speedFactorTurbo;

        float _linkj0Toj1 = 529.5715;
        float _linkj1ToJ2 = 608.45785;
        float _endEffectorLength = 210.1945;

        float _j0Angle;
        float _j1Angle;
        float _j2Angle;

        float _linearControl;
        float _j0Control;
        float _j1Control;
        float _j2Control;
        float _gripperUD;
        float _gripperRL;
        float _gripperOC;

        float _deadmanSwitch;

        float _modeTankAngularInput;
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

        void joyCallback(const rover_msgs::msg::Joy::SharedPtr msg)
        {
            rover_msgs::msg::ArmCmd message;

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

                forwardKinematics(msg);

                inverseKinematics(msg);             
            }
        }

        void forwardKinematics(const rover_msgs::msg::Joy::SharedPtr msg)
        {
            float matRotBase[3][3] = {
                {0, cos(_j0Angle), sin(_j0Angle)},
                {0, sin(_j0Angle), cos(_j0Angle)},
                {1, 0, 0}
            };

            float matRotJ1[3][3] = {
                {cos(_j1Angle), sin(_j1Angle), 0},
                {-sin(_j1Angle), cos(_j1Angle), 0},
                {0, 0, 1}
            };

            float matRotJ2[3][3] = {
                {cos(_j2Angle), sin(_j2Angle), 0},
                {-sin(_j2Angle), cos(_j2Angle), 0},
                {0, 0, 1}  
            };

            



            

        }

        void inverseKinematics(const rover_msgs::msg::Joy::SharedPtr msg)
        {

        }


        
    rclcpp::Subscription<rover_msgs::msg::Joy>::SharedPtr _sub_joy_arm;
    rclcpp::Publisher<rover_msgs::msg::ArmCmd>::SharedPtr _pub_arm_teleop_in;
};

// Teleop class constructor
// =========================================================================   
Teleop::Teleop() : Node("teleop")
{
    this->getParams();

    _sub_joy_arm = this->create_subscription<rover_msgs::msg::Joy>("rover/arm/joy",
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