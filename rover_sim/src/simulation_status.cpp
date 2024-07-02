#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "rover_msgs/msg/joy.hpp"
#include "rover_msgs/msg/arm_cmd.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_broadcaster.h"

class Rover_sim_feedback : public rclcpp::Node
{
public:
    Rover_sim_feedback();

private:

    float _currentLinearPos = 0.0f;
    float _currentJ0Pos = 0.0f;
    float _currentJ1Pos = 0.0f;
    float _currentJ2Pos = 0.0f;
    float _currentGripperLRPos = 0.0f;
    float _currentGripperUDPos = 0.0f;
    float _currentGripperOCPos = 0.0f;

    void simCallback(const rover_msgs::msg::ArmCmd::SharedPtr armCmdMsg)
    {
        _currentLinearPos = armCmdMsg->position[rover_msgs::msg::ArmCmd::LINEAR];
        _currentJ0Pos = armCmdMsg->position[rover_msgs::msg::ArmCmd::J0];
        _currentJ1Pos = armCmdMsg->position[rover_msgs::msg::ArmCmd::J1];
        _currentJ2Pos = armCmdMsg->position[rover_msgs::msg::ArmCmd::J2];
        _currentGripperLRPos = armCmdMsg->position[rover_msgs::msg::ArmCmd::GRIPPERLR];
        _currentGripperUDPos = armCmdMsg->position[rover_msgs::msg::ArmCmd::GRIPPERUD];
        _currentGripperOCPos = armCmdMsg->position[rover_msgs::msg::ArmCmd::GRIPPEROC];

        sensor_msgs::msg::JointState joint_state;
        joint_state.header.stamp = this->now();
        joint_state.name = {
            "base_lin",
            "J0",
            "J1",
            "J2",
            "poignet_ud",
            "poignet_gd",
        };
        
        joint_state.position = {_currentLinearPos, _currentJ0Pos, _currentJ1Pos, _currentJ2Pos, _currentGripperUDPos, _currentGripperLRPos};

        _joint_state_pub->publish(joint_state);
    }

    rclcpp::Subscription<rover_msgs::msg::ArmCmd>::SharedPtr _sub_arm_cmd;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr _joint_state_pub;
    std::shared_ptr<tf2_ros::TransformBroadcaster> _tf_broadcast;
};

Rover_sim_feedback::Rover_sim_feedback() : Node("rover_sim")
{
    _sub_arm_cmd = this->create_subscription<rover_msgs::msg::ArmCmd>("/rover/arm/cmd/in/teleop",
                                                                      1,
                                                                      std::bind(&Rover_sim_feedback::simCallback, this, std::placeholders::_1));
    _joint_state_pub = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 1);
    _tf_broadcast = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Rover_sim_feedback>());
    rclcpp::shutdown();
}