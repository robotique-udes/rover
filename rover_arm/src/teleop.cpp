#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "rover_msgs/msg/joy.hpp"
#include "rover_msgs/msg/propulsion_motor.hpp"
#include "rover_msgs/msg/joy_demux_status.hpp"
#include "rover_msgs/msg/arm_cmd.hpp"
#include "rovus_lib/macros.h"

#include <armadillo>

// =============================================================================
// This node has for goal to control the robotic arm of the rover.
//
// For broader information about the mathematics behind the calculations, please 
// consult the readme for more information
//
// This node sends its calculated messages to the motors and to the simulation
// =============================================================================

#define MAX_JOINT_SPEED 0.0017453f 
#define MAX_LIN_SPEED 0.00017453f  

const float J0x = 0.0f;
const float J0y = 0.0f;
const float J0z = 0.0f;

const float J1x = 0.0f;
const float J1y = 0.0f;
const float J1z = 0.0f;

const float J2x = 0.65f;
const float J2y = 0.0f;
const float J2z = 0.0f;

const float J3x = 0.62f;
const float J3y = 0.0f;
const float J3z = 0.0f;

const float J4x = 0.217f;
const float J4y = 0.0f;
const float J4z = 0.0f;

const float maxJLVelocity = MAX_LIN_SPEED;
const float maxJ0Velocity = MAX_JOINT_SPEED;
const float maxJ1Velocity = MAX_JOINT_SPEED;
const float maxJ2Velocity = MAX_JOINT_SPEED;
const float maxGripperTiltVelocity = MAX_JOINT_SPEED;
const float maxGripperRotVelocity = MAX_JOINT_SPEED;
const float maxGripperCloseVelocity = MAX_LIN_SPEED;

struct sCurrentJointVelocity
{
    float q0, q1, q2, q3, q4;

    sCurrentJointVelocity(float q0_, float q1_, float q2_, float q3_, float q4_)
    {
        q0 = q0_;
        q1 = q1_;
        q2 = q2_;
        q3 = q3_;
        q4 = q4_;
    }
};

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

    // CARTESIAN CONTROL COMMANDS
    //  =========================================================================
    float _xVelocity;
    float _yVelocity;
    float _zVelocity;
    float _alphaVelocity; 
    float _psiVelocity;   

    // CARTESIAN VECTORS AND MATRICES
    //  =========================================================================
    arma::vec5 _desiredCartesianVelocity;
    arma::mat55 _jacobian;
    arma::mat55 _inverseJacobian;
    arma::vec5 _computedJointVelocity;
    arma::vec5 _constrainedJointVelocity;
    arma::vec5 _maxJointVelocity;
    arma::mat _pointPositions;

    sCurrentJointVelocity _currentState = sCurrentJointVelocity(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

    // COMMAND MODES
    //  =========================================================================
    bool _deadmanSwitch;
    bool _gripperMode;
    bool _controlMode;
    bool _controlModeToggle;
    bool _fixedCartesianMode;

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

    arma::mat55 computeJacobian(const sCurrentJointVelocity &state);
    arma::mat computeDirectKinematic(const sCurrentJointVelocity &state);
    arma::vec5 constrainJointVelocity(arma::vec5 jointVelocity, arma::vec5 maxJointVelocity);

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
    _maxJointVelocity = {maxJLVelocity, maxJ0Velocity, maxJ1Velocity, maxJ2Velocity, maxGripperTiltVelocity}; // THIS IS TO BE UPDATED WITH TRUE INDIVIDUAL MAX SPEED - REPLACE WITH CONST EXPRESSIONS
}

void Teleop::joyCallback(const rover_msgs::msg::Joy::SharedPtr joyMsg)
{
    // Mode selection
    // =========================================================================
    _deadmanSwitch = joyMsg->joy_data[rover_msgs::msg::Joy::L1];
    _gripperMode = joyMsg->joy_data[rover_msgs::msg::Joy::R1];
    _fixedCartesianMode = joyMsg->joy_data[rover_msgs::msg::Joy::R1];
    _controlModeToggle = joyMsg->joy_data[rover_msgs::msg::Joy::A];

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
    _xVelocity = joyMsg->joy_data[rover_msgs::msg::Joy::JOYSTICK_LEFT_FRONT] * -1.0f;
    _yVelocity = joyMsg->joy_data[rover_msgs::msg::Joy::CROSS_LEFT] - joyMsg->joy_data[rover_msgs::msg::Joy::CROSS_RIGHT];
    _zVelocity = joyMsg->joy_data[rover_msgs::msg::Joy::JOYSTICK_RIGHT_FRONT];
    _alphaVelocity = joyMsg->joy_data[rover_msgs::msg::Joy::JOYSTICK_LEFT_FRONT];
    _psiVelocity = joyMsg->joy_data[rover_msgs::msg::Joy::JOYSTICK_RIGHT_SIDE];

    // Initialize controls to previous values
    // =========================================================================
    armMsg.position[rover_msgs::msg::ArmCmd::JL] = _currentJLPos;
    armMsg.position[rover_msgs::msg::ArmCmd::J0] = _currentJ0Pos;
    armMsg.position[rover_msgs::msg::ArmCmd::J1] = _currentJ1Pos;
    armMsg.position[rover_msgs::msg::ArmCmd::J2] = _currentJ2Pos;
    armMsg.position[rover_msgs::msg::ArmCmd::GRIPPERTILT] = _currentGripperTilt;
    armMsg.position[rover_msgs::msg::ArmCmd::GRIPPERROT] = _currentGripperRot;
    armMsg.position[rover_msgs::msg::ArmCmd::GRIPPEROPENCLOSE] = _currentGripperState;

    // Control mode toggle
    // =========================================================================
    if (_controlModeToggle)
    {
        _controlMode = !_controlMode;
    }

    // Control logic
    // =========================================================================
    if (_deadmanSwitch)
    {
        if (_controlMode == CARTESIAN)
        {
            if (_fixedCartesianMode)
            {
                _desiredCartesianVelocity = {_xVelocity * 0.0f, _yVelocity * 0.0f, _zVelocity * 0.0f, _alphaVelocity, _psiVelocity};
            }
            else
            {
                _desiredCartesianVelocity = {_xVelocity, _yVelocity, _zVelocity, _alphaVelocity * 0.0f, _psiVelocity * 0.0f};
            }

            _jacobian = computeJacobian(sCurrentJointVelocity(_currentJLPos, _currentJ0Pos, _currentJ1Pos, _currentJ2Pos, _currentGripperTilt));
            _inverseJacobian = arma::inv(_jacobian);

            _computedJointVelocity = _inverseJacobian * _desiredCartesianVelocity;

            _constrainedJointVelocity = constrainJointVelocity(_computedJointVelocity, _maxJointVelocity);

            _currentJLPos += _constrainedJointVelocity(0);
            _currentJ0Pos += _constrainedJointVelocity(1);
            _currentJ1Pos += _constrainedJointVelocity(2);
            _currentJ2Pos += _constrainedJointVelocity(3);
            _currentGripperTilt += _constrainedJointVelocity(4);
            _currentGripperState += _gripperState * maxGripperCloseVelocity;
        }

        if (_controlMode == JOINT)
        {
            if (_gripperMode)
            {
                _currentGripperTilt += _gripperTilt * maxGripperTiltVelocity;
                _currentGripperRot += _gripperRot * maxGripperRotVelocity;
                _currentGripperState = _gripperState * maxGripperCloseVelocity;
            }
            else
            {
                _currentJLPos += _posCmdJL * maxJLVelocity;
                _currentJ0Pos += _posCmdJ0 * maxJ0Velocity * -1.0f;
                _currentJ1Pos += _posCmdJ1 * maxJ1Velocity;
                _currentJ2Pos += _posCmdJ2 * maxJ2Velocity;
            }
        }
    }

    _pub_arm_teleop_in->publish(armMsg);
}

arma::mat55 Teleop::computeJacobian(const sCurrentJointVelocity &state)
{
    arma::mat55 J(arma::fill::zeros);

    float s1 = sin(state.q1);
    float c1 = cos(state.q1);
    float s2 = sin(0.5 * PI - state.q2);
    float c2 = cos(0.5 * PI - state.q2);
    float s23 = sin(0.5 * PI - state.q2 - state.q3);
    float c23 = cos(0.5 * PI - state.q2 - state.q3);
    float s234 = sin(0.5 * PI - state.q2 - state.q3 - state.q4);
    float c234 = cos(0.5 * PI - state.q2 - state.q3 - state.q4);

    J(0, 0) = 0.0f;
    J(0, 1) = J1x * -s1 + J2x * -s1 * c2 + J2z * -s1 * s2 + J3x * -s1 * c23 + J3z * -s1 * s23 - c1 * (J1y + J2y) + J4x * c1 * c234 + J4z * c1 * s234;
    J(0, 2) = J2x * c1 * -s2 + J2z * c1 * c2 + J3x * c1 * -s23 + J3z * c1 * c23 + J4x * s1 * -s234 + J4z * s1 * c234;
    J(0, 3) = J3x * c1 * -s23 + J3z * c1 * c23 + J4x * s1 * -s234 + J4z * s1 * c234;
    J(0, 4) = J4x * s1 * -s234 + J4z * s1 * c234;

    J(1, 0) = 1.0f;
    J(1, 1) = J1x * c1 + -s1 * (J1y + J2y) + J2x * c1 * c2 + J2z * c1 * s2 + J3x * c1 * c23 + J3z * c1 * s23 + J4x * c1 * c234 + J4z * c1 * s234;
    J(1, 2) = J2x * s1 * -s2 + J2z * s1 * c2 + J3x * s1 * -s23 + J3z * s1 * c23 + J4x * s1 * -s234 + J4z * s1 * c234;
    J(1, 3) = J3x * s1 * -s23 + J3z * s1 * c23 + J4x * s1 * -s234 + J4z * s1 * c234;
    J(1, 4) = J4x * s1 * -s234 + J4z * s1 * c234;

    J(2, 0) = 0.0f;
    J(2, 1) = 0.0f;
    J(2, 2) = J2z * -s2 + J3z * -s23 + J4z * -s234 - J2x * c2 - J3x * c23 - J4x * c234;
    J(2, 3) = J3z * -s23 + J4z * -s234 - J3x * c23 - J4x * c234;
    J(2, 4) = J4z * -s234 - J4x * c234;

    J(3, 0) = 0.0f;
    J(3, 1) = 0.0f;
    J(3, 2) = 1.0f;
    J(3, 3) = 1.0f;
    J(3, 4) = 1.0f;

    J(4, 0) = 0.0f;
    J(4, 1) = 1.0f;
    J(4, 2) = 0.0f;
    J(4, 3) = 0.0f;
    J(4, 4) = 0.0f;

    return J;
}

arma::vec5 Teleop::constrainJointVelocity(arma::vec5 jointVelocity, arma::vec5 maxJointVelocity)
{
    arma::vec5 constrainedVelocity = jointVelocity;
    float highestVelocityRatio = 0.0f;

    for (uint8_t i = 0; i < jointVelocity.n_elem; i++)
    {
        float currentVelocityRatio = std::abs(jointVelocity[i]) / maxJointVelocity[i];
        if (currentVelocityRatio > highestVelocityRatio)
        {
            highestVelocityRatio = currentVelocityRatio;
        }
    }

    if (highestVelocityRatio > 1.0f)
    {
        constrainedVelocity /= highestVelocityRatio;
    }

    return constrainedVelocity;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Teleop>());
    rclcpp::shutdown();
}
