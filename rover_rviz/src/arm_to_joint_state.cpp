#include "rover_msgs/msg/arm_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class ArmToJointStateNode : public rclcpp::Node
{

    private:
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr _publisher;
};

int main(int argc_, char** argv_)
{
    rclcpp::init(argc_, argv_);
    std::shared_ptr<ArmToJointStateNode> node = std::make_shared<ArmToJointStateNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}