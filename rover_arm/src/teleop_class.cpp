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

  public:
    Teleop():
        rclcpp::Node("teleop_node"),
        _jointController({TO_UNDERLYING(eJointIndex::JL), TO_UNDERLYING(eJointIndex::J1), TO_UNDERLYING(eJointIndex::J2)})
    {
        _subJoyArm = this->create_subscription<rover_msgs::msg::Joy>("/rover/arm/joy",
                                                                     1,
                                                                     [this](const rover_msgs::msg::Joy& msg)
                                                                     {
                                                                         this->joy_CB(msg);
                                                                     });
    }

    void joy_CB(const rover_msgs::msg::Joy& joyMsg_)
    {
        const uint8_t joyMsgSize = joyMsg_.joy_data.size();
        std::vector<float> joyArray(joyMsgSize);
        std::copy_n(joyMsg_.joy_data.begin(), joyMsgSize, joyArray.begin());

        float cmd = _jointController.setCmd(joyArray);

        RCLCPP_INFO(this->get_logger(), "Send Cmd: %f, Controlled Joint: %d", cmd, _jointController.getControlledJoint());
        // RCLCPP_INFO(this->get_logger(), "CROSS UP STATUS %d", _jointController._buttonStates[KEYBINDING::JOINT_SELECT_INC]);
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Teleop>());
    rclcpp::shutdown();
}
