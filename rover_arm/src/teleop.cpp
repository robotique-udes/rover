#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "rover_msgs/msg/joy.hpp"
#include "rover_msgs/msg/propulsion_motor.hpp"
#include "rover_msgs/msg/joy_demux_status.hpp"
#include "rover_msgs/msg/arm_cmd.hpp"
#include "rovus_lib/macros.h"

#define MAX_JOINT_SPEED 0.0017453f // 10 deg/s - (0.1745 rad/s)

class Teleop : public rclcpp::Node
{
public:
    Teleop();

private:
    // Private members
    //  =========================================================================
    // float _posCmdJL;
    float _posCmdJLRight;
    float _posCmdJLLeft;
    float _posCmdJ0;
    float _posCmdJ1;
    float _posCmdJ2;
    float _gripperTilt;
    float _gripperRot;
    float _gripperOpen;
    float _gripperClose;

    float _deadmanSwitch;
    float _gripperMode;

    float _speedControl = MAX_JOINT_SPEED; // 10 deg/s - (0.1745 rad/s)

    // Set Constrain values
    //  =========================================================================
    float _currentJLPos;
    float _currentJ0Pos;
    float _currentJ1Pos;
    float _currentJ2Pos;
    float _currentGripperTilt;
    float _currentGripperRot;

    // Private methods
    //  =========================================================================
    void joyCallback(const rover_msgs::msg::Joy::SharedPtr joyMsg);

    rclcpp::Subscription<rover_msgs::msg::Joy>::SharedPtr _sub_joy_arm;
    rclcpp::Publisher<rover_msgs::msg::ArmCmd>::SharedPtr _pub_arm_teleop_in;
    rover_msgs::msg::ArmCmd armMsg;
};

// Teleop class constructor
// =========================================================================
Teleop::Teleop() : Node("teleop")
{
    _sub_joy_arm = this->create_subscription<rover_msgs::msg::Joy>("/rover/arm/joy",
                                                                   1,
                                                                   std::bind(&Teleop::joyCallback, this, std::placeholders::_1));
    _pub_arm_teleop_in = this->create_publisher<rover_msgs::msg::ArmCmd>("/rover/arm/cmd/in/teleop", 1);
}

void Teleop::joyCallback(const rover_msgs::msg::Joy::SharedPtr joyMsg)
{
    // Mode selection
    // =========================================================================
    _deadmanSwitch = joyMsg->joy_data[rover_msgs::msg::Joy::L1];
    _gripperMode = joyMsg->joy_data[rover_msgs::msg::Joy::R1];

    // JOINT controls
    // =========================================================================
    _posCmdJLLeft = joyMsg->joy_data[rover_msgs::msg::Joy::L2];
    _posCmdJLRight = joyMsg->joy_data[rover_msgs::msg::Joy::R2];
    _posCmdJ0 = joyMsg->joy_data[rover_msgs::msg::Joy::JOYSTICK_RIGHT_SIDE];
    _posCmdJ1 = joyMsg->joy_data[rover_msgs::msg::Joy::JOYSTICK_LEFT_FRONT];
    _posCmdJ2 = joyMsg->joy_data[rover_msgs::msg::Joy::JOYSTICK_RIGHT_FRONT];

    // Gripper controls
    // =========================================================================
    _gripperTilt = joyMsg->joy_data[rover_msgs::msg::Joy::JOYSTICK_LEFT_FRONT];
    _gripperRot = joyMsg->joy_data[rover_msgs::msg::Joy::JOYSTICK_RIGHT_SIDE];
    _gripperOpen = joyMsg->joy_data[rover_msgs::msg::Joy::R2];
    _gripperClose = joyMsg->joy_data[rover_msgs::msg::Joy::L2];

    // Initialize controls to previous values
    // =========================================================================
    armMsg.position[rover_msgs::msg::ArmCmd::JL] = _currentJLPos;
    armMsg.position[rover_msgs::msg::ArmCmd::J0] = _currentJ0Pos;
    armMsg.position[rover_msgs::msg::ArmCmd::J1] = _currentJ1Pos;
    armMsg.position[rover_msgs::msg::ArmCmd::J2] = _currentJ2Pos;
    armMsg.position[rover_msgs::msg::ArmCmd::GRIPPERTILT] = _currentGripperTilt;
    armMsg.position[rover_msgs::msg::ArmCmd::GRIPPERROT] = _currentGripperRot;

    // Control logic
    // =========================================================================
    if (_deadmanSwitch)
    {
        if (_gripperMode)
        {
            _currentGripperTilt += _gripperTilt * _speedControl;
            _currentGripperRot += _gripperRot * _speedControl;
            armMsg.position[rover_msgs::msg::ArmCmd::GRIPPEROPENCLOSE] = (_gripperOpen - _gripperClose) * _speedControl;
        }
        else
        {
            _currentJLPos += (_posCmdJLLeft - _posCmdJLRight) * _speedControl;
            _currentJ0Pos += _posCmdJ0 * _speedControl * -1.0f;
            _currentJ1Pos += _posCmdJ1 * _speedControl;
            _currentJ2Pos += _posCmdJ2 * _speedControl;
        }
    }

    _pub_arm_teleop_in->publish(armMsg);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Teleop>());
    rclcpp::shutdown();
}
