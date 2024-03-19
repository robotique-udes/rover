#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "rover_msgs/msg/joy.hpp"
#include <map>
#include "rover_msgs/msg/joy_demux_status.hpp"
using std::placeholders::_1;

//Class definition
class Teleop : public rclcpp::Node
{
private:

    void topic_callback(const rover_msgs::msg::Joy::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "I heard");
    }
    
    rclcpp::Subscription<rover_msgs::msg::Joy>::SharedPtr _sub_joy_formated;

public:
    Teleop();
};

//Constructor
Teleop::Teleop() : Node("teleop")
{
    _sub_joy_formated = this->create_subscription<rover_msgs::msg::Joy>("/joy/main/formated",
                                                                         1,
                                                                         std::bind(&Teleop::topic_callback, this, std::placeholders::_1));
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Teleop>());
    rclcpp::shutdown();
    
    return 0;
}
