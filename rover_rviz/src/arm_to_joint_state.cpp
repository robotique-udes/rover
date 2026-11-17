#include "rover_msgs/msg/arm_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <rover_lib2/helpers/constants.hpp>

enum class eJointIndex : size_t
{
    JL,
    J0,
    J1,
    J2,
    GRIPPER_TILT,
    GRIPPER_ROT,
    GRIPPER_CLOSE,
    eLAST
};

class ArmToJointStateNode : public rclcpp::Node
{
    static constexpr const char* TOPIC_JOINT_STATE = "joint_states";
    static constexpr const char* TOPIC_ARM_STATUS = "/rover/arm/joints_status";
    static constexpr float PI = 3.14159265358979;
    static constexpr float HARD_FIX_J1_POS = 0.09;
    static constexpr float HARD_FIX_J2_POS = -0.6;

    public:
        ArmToJointStateNode();

    private:
        void cbArmToJoint(rover_msgs::msg::ArmMsg msg_);
        static constexpr std::string_view getJointName(eJointIndex joint_);
        float getJointPos(eJointIndex joint_, rover_msgs::msg::ArmMsg msg_);

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr _publisher;
        rclcpp::Subscription<rover_msgs::msg::ArmMsg>::SharedPtr _subArmMsg;


};

ArmToJointStateNode::ArmToJointStateNode(): 
    Node("arm_to_joint_state")
{
    _publisher = create_publisher<sensor_msgs::msg::JointState>(TOPIC_JOINT_STATE, QOS_DEFAULT);
    _subArmMsg = create_subscription<rover_msgs::msg::ArmMsg>(TOPIC_ARM_STATUS,
                                                              QOS_DEFAULT,
                                                              [this](const rover_msgs::msg::ArmMsg msg_)
                                                              {
                                                                this->cbArmToJoint(msg_);
                                                              }
                                                              );
    
}

void ArmToJointStateNode::cbArmToJoint(rover_msgs::msg::ArmMsg msg_)
{
    sensor_msgs::msg::JointState jointMsg;

    jointMsg.header.stamp = now();

    for (size_t i=0; i<std::to_underlying(eJointIndex::eLAST); i++)
    {
        jointMsg.name.emplace_back(getJointName(static_cast<eJointIndex>(i)));
        jointMsg.position.emplace_back(getJointPos(static_cast<eJointIndex>(i), msg_));
        jointMsg.velocity.push_back(msg_.current_speed[i]);
    }

    _publisher->publish(jointMsg);
}

constexpr std::string_view ArmToJointStateNode::getJointName(eJointIndex joint_)
{
    switch (joint_)
    {
        case eJointIndex::JL:
            return "JL";
        case eJointIndex::J0:
            return "J0";
        case eJointIndex::J1:
            return "J1";
        case eJointIndex::J2:
            return "J2";
        case eJointIndex::GRIPPER_TILT:
            return "GRIPPER_TILT";
        case eJointIndex::GRIPPER_ROT:
            return "GRIPPER_ROT";
        case eJointIndex::GRIPPER_CLOSE:
            return "GRIPPER_CLOSE";
        case eJointIndex::eLAST:
            return "";
    }
    std::unreachable();
}

float ArmToJointStateNode::getJointPos(eJointIndex joint_, rover_msgs::msg::ArmMsg msg_)
{
    switch (joint_)
    {
        case eJointIndex::JL:
            return msg_.current_position[std::to_underlying(eJointIndex::JL)];
        case eJointIndex::J0:
            return msg_.current_position[std::to_underlying(eJointIndex::J0)];
        case eJointIndex::J1:
            return msg_.current_position[std::to_underlying(eJointIndex::J1)]- (PI/4) + HARD_FIX_J1_POS;
        case eJointIndex::J2:
            return msg_.current_position[std::to_underlying(eJointIndex::J2)]- (PI/2) + HARD_FIX_J2_POS;
        case eJointIndex::GRIPPER_TILT:
            return msg_.current_position[std::to_underlying(eJointIndex::GRIPPER_TILT)];
        case eJointIndex::GRIPPER_ROT:
            return msg_.current_position[std::to_underlying(eJointIndex::GRIPPER_ROT)];
        case eJointIndex::GRIPPER_CLOSE:
            return msg_.current_position[std::to_underlying(eJointIndex::GRIPPER_CLOSE)];
        case eJointIndex::eLAST:
            return 0;
    }
    std::unreachable();
}

int main(int argc_, char** argv_)
{
    rclcpp::init(argc_, argv_);
    std::shared_ptr<ArmToJointStateNode> node = std::make_shared<ArmToJointStateNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}