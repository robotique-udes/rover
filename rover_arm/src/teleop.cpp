#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "rover_msgs/msg/arm_msg.hpp"
#include "rover_msgs/msg/joy.hpp"

#include "rovus_lib/timer.hpp"
#include "rovus_lib/macros.h"

#include "arm_configuration.hpp"
#include "keybinding.hpp"

constexpr std::chrono::milliseconds WATCHDOG_TIMEOUT{500};

class Teleop : public rclcpp::Node
{
public:
    Teleop();
    ~Teleop() {};

private:
    rclcpp::Subscription<rover_msgs::msg::ArmMsg>::SharedPtr _subArmPositions;
    rclcpp::Subscription<rover_msgs::msg::Joy>::SharedPtr _subJoyArm;
    rclcpp::Publisher<rover_msgs::msg::ArmMsg>::SharedPtr _pubArmCmd;
    rclcpp::TimerBase::SharedPtr _armHeartbeatTimer;

    bool _currentPoseFailure = false;
    std::chrono::steady_clock::time_point _lastPositionData;

    void positionCallback(const rover_msgs::msg::ArmMsg::SharedPtr positionMsg);
    void joyCallback(const rover_msgs::msg::Joy::SharedPtr positionMsg);

    void watchdog(bool& rLostHeartbeat);
};

void Teleop::joyCallback(const rover_msgs::msg::Joy::SharedPtr positionMsg)
{
    // Publish empty data for now
    rover_msgs::msg::ArmMsg msg;
    for (uint8_t i = 0; i < 7; i++)
    {
        msg.data[i] = 0;
    }

    _pubArmCmd->publish(msg);

    // RCLCPP_INFO(this->get_logger(), "Current Pose Failure Status: %s", _currentPoseFailure ? "True" : "False");

}

void Teleop::positionCallback(const rover_msgs::msg::ArmMsg::SharedPtr positionMsg)
{
    _lastPositionData = std::chrono::steady_clock::now();
    bool _currentPoseFailure = false;
}

void Teleop::watchdog(bool& rLostHeartbeat)
{
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - _lastPositionData);
            
        if (elapsed > WATCHDOG_TIMEOUT) 
        {
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(), 
                *this->get_clock(), 
                500,
                "Arm watchdog has been triggered!"
            );
            rLostHeartbeat = true;
        }
}

Teleop::Teleop() : Node("teleop")
{
    _subArmPositions = this->create_subscription<rover_msgs::msg::ArmMsg>("/rover/arm/status/current_positions",
                                                                1,
                                                                [this](const rover_msgs::msg::ArmMsg::SharedPtr msg)
                                                                { this->positionCallback(msg); });
                                                                
    _subJoyArm = this->create_subscription<rover_msgs::msg::Joy>("/rover/arm/joy",
                                                                1,
                                                                [this](const rover_msgs::msg::Joy::SharedPtr msg)
                                                                { this->joyCallback(msg); });

    _pubArmCmd = this->create_publisher<rover_msgs::msg::ArmMsg>("/rover/arm/cmd/goal_speed", 1);

    _armHeartbeatTimer = this->create_wall_timer(std::chrono::milliseconds(500),
                                                                [this]()
                                                                { this->watchdog(_currentPoseFailure); });

}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Teleop>());
    rclcpp::shutdown();
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Teleop>());
    rclcpp::shutdown();
}
