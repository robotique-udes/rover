#include "joy_manager.hpp"
#include "keybinding_science.hpp"

#include <rover_msgs/msg/science_cmd.hpp>
#include <rover_msgs/msg/joy.hpp>

#include <rover_lib2/helpers/time.hpp>
#include <rover_lib2/helpers/loop_timer.hpp>
#include <rover_lib2/helpers/macros.hpp>
#include <rover_lib2/helpers/constants.hpp>

#include <rclcpp/rclcpp.hpp>
#include <utility>

class Teleop : public rclcpp::Node
{
    static constexpr const char* TOPIC_JOY_SCIENCE = "/base/joy/science";
    static constexpr const char* TOPIC_SCIENCE_CMD = "/rover/science/cmd";

    static constexpr const float LIN_ACT_SPEED_FACTOR = 100F;

  private:
    rclcpp::Subscription<rover_msgs::msg::Joy>::SharedPtr _subJoyScience;
    rclcpp::Publisher<rover_msgs::msg::ScienceCmd>::SharedPtr _pubScienceCmd;

    JoyManager _joyManager;

  public:
    Teleop():
        rclcpp::Node("teleop_node"),
        _joyManager()
    {
        _subJoyScience = this->create_subscription<rover_msgs::msg::Joy>(TOPIC_JOY_SCIENCE,
                                                                         QOS_DEFAULT,
                                                                         [this](const rover_msgs::msg::Joy& joyMsg_)
                                                                         {
                                                                             this->joy_CB(joyMsg_);
                                                                         });
        _pubScienceCmd = this->create_publisher<rover_msgs::msg::ScienceCmd>(TOPIC_SCIENCE_CMD, QOS_DEFAULT);
    }

    void joy_CB(const rover_msgs::msg::Joy& joyMsg_)
    {
        const size_t joyMsgSize = joyMsg_.joy_data.size();
        std::array<float, TO_UNDERLYING(Constants::Keybinds::eJoyInput::eLAST)> joyArray = {};
        std::copy_n(joyMsg_.joy_data.begin(), joyMsgSize, joyArray.begin());
        rover_msgs::msg::ScienceCmd scienceCmd;

        _joyManager.updateJoyArray(joyArray);

        if (this->_joyManager.isPressed(KEYBINDINGS::DEADMAN_SWITCH))
        {
            if (this->_joyManager.isPressed(KEYBINDINGS::LINEAR_ACT_UP))
            {
                scienceCmd.target_speed[rover_msgs::msg::ScienceCmd::LINEAR_ACT] = LIN_ACT_SPEED_FACTOR;
            }
            else if (this->_joyManager.isPressed(KEYBINDINGS::LINEAR_ACT_DOWN))
            {
                scienceCmd.target_speed[rover_msgs::msg::ScienceCmd::LINEAR_ACT] = LIN_ACT_SPEED_FACTOR * -1.0F;
            }
            else
            {
                scienceCmd.target_speed[rover_msgs::msg::ScienceCmd::LINEAR_ACT] = 0.0F;
            }

            if (this->_joyManager.isPressed(KEYBINDINGS::EXCAVATOR))
            {
                scienceCmd.target_speed[rover_msgs::msg::ScienceCmd::EXCAVATOR] = 1.0F;
            }
            else
            {
                scienceCmd.target_speed[rover_msgs::msg::ScienceCmd::EXCAVATOR] = 0.0F;
            }

            if (this->_joyManager.isPressed(KEYBINDINGS::BEAK_HOME))
            {
                scienceCmd.target_speed[rover_msgs::msg::ScienceCmd::BEAK] = 0.0F * 2 * std::numbers::pi_v<float>;
            }
            else if (this->_joyManager.isPressed(KEYBINDINGS::BEAK_POUR))
            {
                scienceCmd.target_speed[rover_msgs::msg::ScienceCmd::BEAK] = 30.0F * 2.0F * std::numbers::pi_v<float>;
            }
            else if (this->_joyManager.isPressed(KEYBINDINGS::BEAK_DUMP))
            {
                scienceCmd.target_speed[rover_msgs::msg::ScienceCmd::BEAK] = 180.0F * 2.0F * std::numbers::pi_v<float>;
            }
            else
            {
                scienceCmd.target_speed[rover_msgs::msg::ScienceCmd::BEAK] = 0.0F;
            }

            if (this->_joyManager.isPressed(KEYBINDINGS::CARROUSEL))
            {
                scienceCmd.target_speed[rover_msgs::msg::ScienceCmd::CARROUSEL] = 1.0F;
            }
            else
            {
                scienceCmd.target_speed[rover_msgs::msg::ScienceCmd::CARROUSEL] = 0.0F;
            }

        }
    }
    this->_pubScienceCmd->publish(scienceCmd);
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Teleop>());
    rclcpp::shutdown();
}
