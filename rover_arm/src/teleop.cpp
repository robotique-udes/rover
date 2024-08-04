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

    enum eJoints
    {
        JL = 0,
        J0 = 1,
        J1 = 2,
        J2 = 3,
        GRIPPER_TILT = 4,
        GRIPPER_ROT = 5,
        GRIPPER_CLOSE = 6
    };

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
    arma::vec _maxJointVelocity;
    arma::vec5 _cartesianMaxJointVelocity;
    arma::mat _pointPositions;

    sCurrentJointVelocity _currentState = sCurrentJointVelocity(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

    // COMMAND MODES
    //  =========================================================================
    bool _deadmanSwitch;
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
    arma::vec5 constrainJointVelocity(arma::vec5 jointVelocity, arma::vec maxJointVelocity);

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
    _fixedCartesianMode = joyMsg->joy_data[rover_msgs::msg::Joy::R1];
    _controlModeToggle = joyMsg->joy_data[rover_msgs::msg::Joy::Y];

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

    // Initialize controls to previous values
    // =========================================================================
    armMsg.position[rover_msgs::msg::ArmCmd::JL] = _currentJointPos[JL];
    armMsg.position[rover_msgs::msg::ArmCmd::J0] = _currentJointPos[J0];
    armMsg.position[rover_msgs::msg::ArmCmd::J1] = _currentJointPos[J1];
    armMsg.position[rover_msgs::msg::ArmCmd::J2] = _currentJointPos[J2];
    armMsg.position[rover_msgs::msg::ArmCmd::GRIPPERTILT] = _currentJointPos[GRIPPER_TILT];
    armMsg.position[rover_msgs::msg::ArmCmd::GRIPPERROT] = _currentJointPos[GRIPPER_ROT];
    armMsg.position[rover_msgs::msg::ArmCmd::GRIPPEROPENCLOSE] = _currentJointPos[GRIPPER_CLOSE];

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
            _desiredCartesianVelocity = {_xVelocity, _yVelocity, _zVelocity, _alphaVelocity, _psiVelocity};

            _jacobian = computeJacobian(sCurrentJointVelocity(_currentJointPos[JL], _currentJointPos[J0], _currentJointPos[J1], _currentJointPos[J2], _currentJointPos[GRIPPER_TILT]));
            _inverseJacobian = arma::inv(_jacobian);

            _computedJointVelocity = _inverseJacobian * _desiredCartesianVelocity;

            _constrainedJointVelocity = constrainJointVelocity(_computedJointVelocity, _cartesianMaxJointVelocity);

            _currentJointPos[JL] += _constrainedJointVelocity(0);
            _currentJointPos[J0] += _constrainedJointVelocity(1);
            _currentJointPos[J1] += _constrainedJointVelocity(2);
            _currentJointPos[J2] += _constrainedJointVelocity(3);
            _currentJointPos[GRIPPER_TILT] += _constrainedJointVelocity(4);

            if (_gripperState)
            {
                _currentJointPos[GRIPPER_CLOSE] += _gripperState * _maxJointVelocity[GRIPPER_CLOSE];
            }
            else if (_currentJointPos[GRIPPER_CLOSE] > 0.0f)
            {
                _currentJointPos[GRIPPER_CLOSE] -= 1.0f * _maxJointVelocity[GRIPPER_CLOSE];
            }
        }

        if (_controlMode == JOINT)
        {
            if (_changeSelectedJoint == 0)
            {
                _updatedJoint = false;
            }

            if (_changeSelectedJoint != 0 && !_updatedJoint)
            {
                if (_changeSelectedJoint > 0)
                {
                    if (_selectedJoint == GRIPPER_TILT)
                    {
                        RCLCPP_WARN(LOGGER, "YOU HAVE REACHED THE LAST JOINT AND CAN NO LONGER INCREMENT");
                    }
                    else
                    {
                        _selectedJoint++;
                        _updatedJoint = true;
                    }
                }
                else if (_changeSelectedJoint < 0)
                {
                    if (_selectedJoint == JL)
                    {
                        RCLCPP_WARN(LOGGER, "YOU HAVE REACHED THE BASE JOINT AND CAN NO LONGER DECREMENT");
                    }
                    else
                    {
                        _selectedJoint--;
                        _updatedJoint = true;
                    }
                }
            }

            _currentJointPos[_selectedJoint] += _jointPosCmd * _maxJointVelocity[_selectedJoint];
            _currentJointPos[GRIPPER_ROT] += _gripperRot * _maxJointVelocity[GRIPPER_ROT];

            if (_gripperState)
            {
                _currentJointPos[GRIPPER_CLOSE] += _gripperState * _maxJointVelocity[GRIPPER_CLOSE];
            }
            else if (_currentJointPos[GRIPPER_CLOSE] > 0.0f)
            {
                _currentJointPos[GRIPPER_CLOSE] -= 1.0f * _maxJointVelocity[GRIPPER_CLOSE];
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

arma::vec5 Teleop::constrainJointVelocity(arma::vec5 jointVelocity, arma::vec maxJointVelocity)
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
