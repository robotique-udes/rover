#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "rover_msgs/msg/joy.hpp"
#include "rover_msgs/msg/science_control.hpp"
#include "rover_msgs/msg/joy_demux_status.hpp"

// Class definition
class Science : public rclcpp::Node
{
public:
    Science();

private:
    // Private members

    float _drillUp;
    float _drillDown;

    float _drillIn;
    float _drillOut;

    // Private methods
    void joyCallback(const rover_msgs::msg::Joy::SharedPtr msg)
    {
        rover_msgs::msg::ScienceControl message;

        _drillUp = msg->joy_data[rover_msgs::msg::Joy::CROSS_UP];
        _drillDown = msg->joy_data[rover_msgs::msg::Joy::CROSS_DOWN];

        _drillIn = msg->joy_data[rover_msgs::msg::Joy::A];
        _drillOut = msg->joy_data[rover_msgs::msg::Joy::Y];

        message.cmd[rover_msgs::msg::ScienceControl::DRILL_UP] = _drillUp;
        message.cmd[rover_msgs::msg::ScienceControl::DRILL_DOWN] = _drillDown;
        
        message.drill[rover_msgs::msg::ScienceControl::DRILL_IN] = _drillIn;
        message.drill[rover_msgs::msg::ScienceControl::DRILL_OUT] = _drillOut;
    

        _pub_science_in->publish(message);
    }

    rclcpp::Subscription<rover_msgs::msg::Joy>::SharedPtr _sub_arm_joy;
    rclcpp::Publisher<rover_msgs::msg::ScienceControl>::SharedPtr _pub_science_in;
};

// Constructor
Science::Science() : Node("Science")
{
    _sub_arm_joy = this->create_subscription<rover_msgs::msg::Joy>("/rover/arm/joy",
                                                                        1,
                                                                        std::bind(&Science::joyCallback, this, std::placeholders::_1));

    _pub_science_in = this->create_publisher<rover_msgs::msg::ScienceControl>("/rover/arm/cmd/in/science", 1);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Science>());
    rclcpp::shutdown();

    return 0;
}
