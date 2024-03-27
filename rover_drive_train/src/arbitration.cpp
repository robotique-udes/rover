#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "rover_msgs/msg/joy.hpp"
#include "rover_msgs/msg/propulsion_motor.hpp"
#include "rover_msgs/msg/joy_demux_status.hpp"

//Class definition
class Arbitration : public rclcpp::Node
{
    public:
        Arbitration();

    private:

        void arbCallback(const rover_msgs::msg::PropulsionMotor & msg)
        {

        }

    rclcpp::Subscription<rover_msgs::msg::PropulsionMotor>::SharedPtr _sub_motor_cmd;
    rclcpp::Subscription<rover_msgs::msg::PropulsionMotor>::SharedPtr _sub_security;

};

Arbitration::Arbitration() : Node("arbitration")
{
    _sub_motor_cmd = this->create_subscription<rover_msgs::msg::PropulsionMotor>("/rover/drive_train/cmd/in/teleop",
                                                                                    1,
                                                                                    std::bind(&Arbitration::arbCallback, this, std::placeholders::_1));
    _sub_security = this->create_subscription<rover_msgs::msg::PropulsionMotor>("/rover/drive_train/cmd/in/security",
                                                                                    1,
                                                                                    std::bind(&Arbitration::arbCallback, this, std::placeholders::_1));
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Arbitration>());
    rclcpp::shutdown();
    
    return 0;
}
