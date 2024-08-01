#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "rover_msgs/msg/joy.hpp"
#include "rover_msgs/msg/propulsion_motor.hpp"
#include "rover_msgs/msg/joy_demux_status.hpp"
#include "rover_msgs/msg/arm_cmd.hpp"
#include "rovus_lib/macros.h"

#include <armadillo>

class Teleop : public rclcpp::Node
{
public:
    Teleop();

private:
    // JOINT CONTROL COMMANDS
    //  =========================================================================
    uint8_t _selectedJoint;
    int8_t _changeSelectedJoint;
    float _jointPosCmd;
    bool _updatedJoint;
    float _gripperRot;
    float _gripperState;

    bool _deadmanSwitch;
    bool _gripperMode;

    enum eControlMode
    {
        CARTESIAN = 0,
        JOINT = 1
    };

    // Set Constrain values
    //  =========================================================================
    float _currentJLPos;
    float _currentJ0Pos;
    float _currentJ1Pos;
    float _currentJ2Pos;
    float _currentGripperTilt;
    float _currentGripperRot;
    float _currentGripperState;

    arma::vec _currentJointPos;

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

    _controlMode = JOINT;
    _selectedJoint = 0;
    _updatedJoint = false;
    _currentJointPos = {_currentJLPos, _currentJ0Pos, _currentJ1Pos, _currentJ2Pos, _currentGripperTilt, _currentGripperRot, _currentGripperState};
    _maxJointVelocity = {maxJLVelocity, maxJ0Velocity, maxJ1Velocity, maxJ2Velocity, maxGripperTiltVelocity, maxGripperRotVelocity, maxGripperCloseVelocity}; // THIS IS TO BE UPDATED WITH TRUE INDIVIDUAL MAX SPEED - REPLACE WITH CONST EXPRESSIONS
    _cartesianMaxJointVelocity = {maxJLVelocity, maxJ0Velocity, maxJ1Velocity, maxJ2Velocity, maxGripperTiltVelocity};                                        // THIS IS TO BE UPDATED WITH TRUE INDIVIDUAL MAX SPEED - REPLACE WITH CONST EXPRESSIONS
}

void Teleop::joyCallback(const rover_msgs::msg::Joy::SharedPtr joyMsg)
{
    // Mode selection
    // =========================================================================
    _deadmanSwitch = joyMsg->joy_data[rover_msgs::msg::Joy::L1];
    _gripperMode = joyMsg->joy_data[rover_msgs::msg::Joy::R1];

    // JOINT controls
    // =========================================================================
    _jointPosCmd = joyMsg->joy_data[rover_msgs::msg::Joy::R2] - joyMsg->joy_data[rover_msgs::msg::Joy::L2];
    _changeSelectedJoint = joyMsg->joy_data[rover_msgs::msg::Joy::CROSS_UP] - joyMsg->joy_data[rover_msgs::msg::Joy::CROSS_DOWN];
    _gripperRot = joyMsg->joy_data[rover_msgs::msg::Joy::JOYSTICK_RIGHT_SIDE];
    _gripperState = joyMsg->joy_data[rover_msgs::msg::Joy::A];

    // CARTESIAN controls
    // =========================================================================
    _xVelocity = joyMsg->joy_data[rover_msgs::msg::Joy::JOYSTICK_LEFT_FRONT] * -1.0f;
    _yVelocity = joyMsg->joy_data[rover_msgs::msg::Joy::CROSS_LEFT] - joyMsg->joy_data[rover_msgs::msg::Joy::CROSS_RIGHT];
    _zVelocity = joyMsg->joy_data[rover_msgs::msg::Joy::JOYSTICK_RIGHT_FRONT];
    _alphaVelocity = joyMsg->joy_data[rover_msgs::msg::Joy::CROSS_UP] - joyMsg->joy_data[rover_msgs::msg::Joy::CROSS_DOWN];
    _psiVelocity = joyMsg->joy_data[rover_msgs::msg::Joy::B] - joyMsg->joy_data[rover_msgs::msg::Joy::X];

    // CARTESIAN controls
    // =========================================================================
    _xVelocity = joyMsg->joy_data[rover_msgs::msg::Joy::JOYSTICK_LEFT_FRONT];
    _yVelocity = joyMsg->joy_data[rover_msgs::msg::Joy::L2] - joyMsg->joy_data[rover_msgs::msg::Joy::R2];
    _zVelocity = joyMsg->joy_data[rover_msgs::msg::Joy::JOYSTICK_RIGHT_FRONT];
    _alphaVelocity = joyMsg->joy_data[rover_msgs::msg::Joy::JOYSTICK_LEFT_FRONT];
    _psiVelocity = joyMsg->joy_data[rover_msgs::msg::Joy::L2] - joyMsg->joy_data[rover_msgs::msg::Joy::R2];

    // Initialize controls to previous values
    // =========================================================================
    armMsg.position[rover_msgs::msg::ArmCmd::JL] = _currentJointPos[JL];
    armMsg.position[rover_msgs::msg::ArmCmd::J0] = _currentJointPos[J0];
    armMsg.position[rover_msgs::msg::ArmCmd::J1] = _currentJointPos[J1];
    armMsg.position[rover_msgs::msg::ArmCmd::J2] = _currentJointPos[J2];
    armMsg.position[rover_msgs::msg::ArmCmd::GRIPPERTILT] = _currentJointPos[GRIPPER_TILT];
    armMsg.position[rover_msgs::msg::ArmCmd::GRIPPERROT] = _currentJointPos[GRIPPER_ROT];
    armMsg.position[rover_msgs::msg::ArmCmd::GRIPPEROPENCLOSE] = _currentJointPos[GRIPPER_CLOSE];

    // Control logic
    // =========================================================================
    if (_deadmanSwitch)
    {
        if (_controlMode == CARTESIAN)
        {
            _currentGripperTilt += _gripperTilt * MAX_JOINT_SPEED;
            _currentGripperRot += _gripperRot * MAX_JOINT_SPEED;
            armMsg.position[rover_msgs::msg::ArmCmd::GRIPPEROPENCLOSE] = _gripperState * MAX_JOINT_SPEED;
        }

        if (_controlMode == JOINT)
        {
            _currentJLPos += _posCmdJL * MAX_JOINT_SPEED;
            _currentJ0Pos += _posCmdJ0 * MAX_JOINT_SPEED * -1.0f;
            _currentJ1Pos += _posCmdJ1 * MAX_JOINT_SPEED;
            _currentJ2Pos += _posCmdJ2 * MAX_JOINT_SPEED;
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
