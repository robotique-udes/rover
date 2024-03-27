#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "rover_msgs/msg/joy.hpp"
#include "rover_msgs/msg/propulsion_motor.hpp"
#include "rover_msgs/msg/joy_demux_status.hpp"

//Class definition
class Watchdog : public rclcpp::Node
{
    public:
        Watchdog();

    private:

        void arbCallback(const rover_msgs::msg::PropulsionMotor & msg)
        {

        }

    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr _sub_base_heartbeat;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr _sub_rover_heartbeat;

    rclcpp::Publisher<rover_msgs::msg::PropulsionMotor>::SharedPtr _pub_security_cmd;

};

Watchdog::Watchdog() : Node("watchdog")
{
    _sub_base_heartbeat = this->create_subscription<std_msgs::msg::Empty>("/base/heartbeat",
                                                                                    1,
                                                                                    std::bind(&Watchdog::arbCallback, this, std::placeholders::_1));
    _sub_rover_heartbeat = this->create_subscription<std_msgs::msg::Empty>("/rover/heartbeat",
                                                                                        1,
                                                                                        std::bind(&Watchdog::arbCallback, this, std::placeholders::_1));
    
    _pub_security_cmd = this->create_publisher<rover_msgs::msg::PropulsionMotor>("rover/drive_train/cmd/in/security", 1);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Watchdog>());
    rclcpp::shutdown();
    
    return 0;
}
