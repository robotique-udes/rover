#include "rclcpp/rclcpp.hpp"

#include "rover_msgs/msg/arm_msg.hpp"
#include "rover_msgs/msg/joy.hpp"

#include "rovus_lib/timer.hpp"
#include "rovus_lib/macros.h"
#include "arm_configuration.hpp"
#include "keybinding.hpp"

#include <joint_controller.hpp>

class Teleop : public rclcpp::Node
{
  public:
  private:
    rclcpp::Subscription<rover_msgs::msg::Joy>::SharedPtr _subJoyArm;
    rclcpp::Publisher<rover_msgs::msg::ArmMsg>::SharedPtr _pubArmCmd;

    JointController _jointController;
    JointController _gripperController;

  public:
    Teleop():
        rclcpp::Node("teleop_node"),
        _jointController({TO_UNDERLYING(eJointIndex::JL), TO_UNDERLYING(eJointIndex::J1), TO_UNDERLYING(eJointIndex::J2)}),
        _gripperController({TO_UNDERLYING(eJointIndex::GRIPPER_CLOSE), TO_UNDERLYING(eJointIndex::GRIPPER_ROT)})
    {
        _subJoyArm = this->create_subscription<rover_msgs::msg::Joy>("/rover/arm/joy",
                                                                     1,
                                                                     [this](const rover_msgs::msg::Joy& msg)
                                                                     {
                                                                         this->joy_CB(msg);
                                                                     });

        _pubArmCmd = this->create_publisher<rover_msgs::msg::ArmMsg>("/rover/arm/cmd/goal_speed", 1);
    }

    void joy_CB(const rover_msgs::msg::Joy& joyMsg_)
    {
        const uint8_t joyMsgSize = joyMsg_.joy_data.size();
        std::vector<float> joyArray(joyMsgSize);
        std::copy_n(joyMsg_.joy_data.begin(), joyMsgSize, joyArray.begin());
        rover_msgs::msg::ArmMsg armMsg;

        // JOINT CONTROL
        if (_jointController.isSelected(joyArray[KEYBINDINGS_EMILE::JOINT::JOINT_SELECTION::INC],
                                        KEYBINDINGS_EMILE::JOINT::JOINT_SELECTION::INC))
        {
            _jointController.setControlledJoint(KEYBINDINGS_EMILE::JOINT::JOINT_SELECTION::INC);
        }
        if (_jointController.isSelected(joyArray[KEYBINDINGS_EMILE::JOINT::JOINT_SELECTION::DEC],
                                        KEYBINDINGS_EMILE::JOINT::JOINT_SELECTION::DEC))
        {
            _jointController.setControlledJoint(KEYBINDINGS_EMILE::JOINT::JOINT_SELECTION::DEC);
        }

        armMsg.data[_jointController.getControlledJoint()] = _jointController.setCmd(joyArray);

        // GRIPPER CONTROL
        armMsg.data[_gripperController.getControlledJoint()] = _gripperController.setCmd(joyArray);

        _pubArmCmd->publish(armMsg);
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Teleop>());
    rclcpp::shutdown();
}
