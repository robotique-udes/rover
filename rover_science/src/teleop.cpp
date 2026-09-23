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

enum class eServoPos : uint8_t
{
    HOME = 0,
    POUR,
    DUMP,
};

class Teleop : public rclcpp::Node
{
    static constexpr const char* TOPIC_JOY_SCIENCE = "/base/joy/science";
    static constexpr const char* TOPIC_SCIENCE_CMD = "/rover/science/cmd";

    static constexpr const float LIN_ACT_SPEED_FACTOR = 100.0F;

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
                scienceCmd.lin_speed = LIN_ACT_SPEED_FACTOR;
            }
            else if (this->_joyManager.isPressed(KEYBINDINGS::LINEAR_ACT_DOWN))
            {
                scienceCmd.lin_speed = LIN_ACT_SPEED_FACTOR * -1.0F;
            }
            else
            {
                scienceCmd.lin_speed = 0.0F;
            }

            if (this->_joyManager.isPressed(KEYBINDINGS::EXCAVATOR))
            {
                scienceCmd.grinder_on = true;
            }
            else
            {
                scienceCmd.grinder_on = false;
            }

            if (this->_joyManager.isPressed(KEYBINDINGS::BEAK_POUR))
            {
                scienceCmd.beak_pos = static_cast<uint8_t>(eServoPos::POUR);
            }
            else if (this->_joyManager.isPressed(KEYBINDINGS::BEAK_DUMP))
            {
                scienceCmd.beak_pos = static_cast<uint8_t>(eServoPos::DUMP);
            }
            else
            {
                scienceCmd.beak_pos = static_cast<uint8_t>(eServoPos::HOME);
            }

            if (this->_joyManager.isPressed(KEYBINDINGS::CARROUSEL))
            {
                scienceCmd.carrousel_on = true;
            }
            else
            {
                scienceCmd.carrousel_on = false;
            }
        }
        this->_pubScienceCmd->publish(scienceCmd);
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Teleop>());
    rclcpp::shutdown();
}
