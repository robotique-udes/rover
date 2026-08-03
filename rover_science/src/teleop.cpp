#include "joy_manager.hpp"
#include "keybinding_science.hpp"

#include <rover_msgs/msg/science_msg.hpp>
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

    static constexpr const float LIN_ACT_SPEED_FACTOR = 0.5F;

  private:
    rclcpp::Subscription<rover_msgs::msg::Joy>::SharedPtr _subJoyScience;
    rclcpp::Publisher<rover_msgs::msg::ScienceMsg>::SharedPtr _pubScienceCmd;

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
        _pubScienceCmd = this->create_publisher<rover_msgs::msg::ScienceMsg>(TOPIC_SCIENCE_CMD, QOS_DEFAULT);
    }

    void joy_CB(const rover_msgs::msg::Joy& joyMsg_)
    {
        const size_t joyMsgSize = joyMsg_.joy_data.size();
        std::array<float, TO_UNDERLYING(Constants::Keybinds::eJoyInput::eLAST)> joyArray = {};
        std::copy_n(joyMsg_.joy_data.begin(), joyMsgSize, joyArray.begin());
        rover_msgs::msg::ScienceMsg scienceMsg;

        _joyManager.updateJoyArray(joyArray);

        if (this->_joyManager.isPressed(KEYBINDINGS::DEADMAN_SWITCH))
        {
            if (this->_joyManager.isPressed(KEYBINDINGS::LINEAR_ACT_UP))
            {
                scienceMsg.target_speed[rover_msgs::msg::ScienceMsg::LINEAR_ACT] = LIN_ACT_SPEED_FACTOR;
            }
            else if (this->_joyManager.isPressed(KEYBINDINGS::LINEAR_ACT_DOWN))
            {
                scienceMsg.target_speed[rover_msgs::msg::ScienceMsg::LINEAR_ACT] = LIN_ACT_SPEED_FACTOR * -1.0F;
            }
            else
            {
                scienceMsg.target_speed[rover_msgs::msg::ScienceMsg::LINEAR_ACT] = 0.0F;
            }

            if (this->_joyManager.isPressed(KEYBINDINGS::EXCAVATOR))
            {
                scienceMsg.target_speed[rover_msgs::msg::ScienceMsg::EXCAVATOR] = 1.0F;
            }
            else
            {
                scienceMsg.target_speed[rover_msgs::msg::ScienceMsg::EXCAVATOR] = 0.0F;
            }

            if (this->_joyManager.isPressed(KEYBINDINGS::BEAK))
            {
                scienceMsg.target_speed[rover_msgs::msg::ScienceMsg::BEAK] = 1.0F;
            }
            else
            {
                scienceMsg.target_speed[rover_msgs::msg::ScienceMsg::BEAK] = 0.0F;
            }
            if (this->_joyManager.isPressed(KEYBINDINGS::CARROUSEL))
            {
                scienceMsg.target_speed[rover_msgs::msg::ScienceMsg::CARROUSEL] = 1.0F;
            }
            else
            {
                scienceMsg.target_speed[rover_msgs::msg::ScienceMsg::CARROUSEL] = 0.0F;
            }

            this->_pubScienceCmd->publish(scienceMsg);
        }
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Teleop>());
    rclcpp::shutdown();
}
