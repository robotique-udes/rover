#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "rover_msgs/msg/joy.hpp"
#include <chrono>
#include "rover_msgs/msg/joy_demux_status.hpp"
using std::placeholders::_1;
using namespace std::chrono_literals;

//Class definition
class Teleop : public rclcpp::Node
{
private:

    void topic_callback(const rover_msgs::msg::Joy::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "I heard");

        auto message = rover_msgs::msg::Joy();
        message.joy_data = msg->joy_data;

        RCLCPP_INFO(this->get_logger(), "Joy_LF: %f", msg->joy_data[rover_msgs::msg::Joy::JOYSTICK_LEFT_FRONT]);
        _pub_teleop_in->publish(message);
    }
    
    rclcpp::Subscription<rover_msgs::msg::Joy>::SharedPtr _sub_joy_formated;
    rclcpp::Publisher<rover_msgs::msg::Joy>::SharedPtr _pub_teleop_in;

public:
    Teleop();
};

//Constructor
Teleop::Teleop() : Node("teleop")
{
    _sub_joy_formated = this->create_subscription<rover_msgs::msg::Joy>("/joy/main/formated",
                                                                         1,
                                                                         std::bind(&Teleop::topic_callback, this, std::placeholders::_1));

    _pub_teleop_in = this->create_publisher<rover_msgs::msg::Joy>("/rover/drive_train/cmd/in/teleop", 1);
                                                                   
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Teleop>());
    rclcpp::shutdown();
    
    return 0;
}
