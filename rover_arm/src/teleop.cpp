#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "rover_msgs/msg/joy.hpp"
#include "rover_msgs/msg/propulsion_motor.hpp"
#include "rover_msgs/msg/joy_demux_status.hpp"
#include "rover_msgs/msg/arm_cmd.hpp"
#include "rovus_lib/macros.h"

#include <armadillo>

#define MAX_JOINT_SPEED 0.0017453f // 10 deg/s - (0.1745 rad/s)
#define MAX_CART_JOINT_SPEED 0.17453f

struct RobotState
{
    float q0, q1, q2, q3, q4;
    float J0x = 0.0f, J0y = 0.0f, J0z = 0.0f;
    float J1x = 0.0f, J1y = 0.0f, J1z = 0.0f;
    float J2x = 0.65f, J2y = 0.0f, J2z = 0.0f;
    float J3x = 0.61f, J3y = 0.0f, J3z = 0.0f;
    float J4x = 0.217f, J4y = 0.0f, J4z = 0.0f;

    RobotState(float q0_, float q1_, float q2_, float q3_, float q4_)
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
    float _alphaVelocity; // Gripper rotation around Y axis with constant XYS pos
    float _psiVelocity;   // Gripper rotation around Z axis with constant XYS pos

    // CARTESIAN VECTORS AND MATRICES
    //  =========================================================================
    arma::vec5 _desiredCartesianVelocity;
    arma::mat55 _jacobian;
    arma::mat55 _inverseJacobian;
    arma::vec5 _computedJointVelocity;
    arma::vec5 _constrainedJointVelocity;
    arma::vec5 _maxJointVelocity;
    arma::mat _pointPositions;

    RobotState _currentState = RobotState(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

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

    arma::mat55 computeJacobian(const RobotState &state);
    arma::mat computeDirectKinematic(const RobotState &state);
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
    _maxJointVelocity = {MAX_JOINT_SPEED, MAX_JOINT_SPEED, MAX_JOINT_SPEED, MAX_JOINT_SPEED, MAX_JOINT_SPEED};
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
    _yVelocity = joyMsg->joy_data[rover_msgs::msg::Joy::L2] - joyMsg->joy_data[rover_msgs::msg::Joy::R2];
    _zVelocity = joyMsg->joy_data[rover_msgs::msg::Joy::JOYSTICK_RIGHT_FRONT];
    _alphaVelocity = joyMsg->joy_data[rover_msgs::msg::Joy::JOYSTICK_LEFT_FRONT];
    _psiVelocity = joyMsg->joy_data[rover_msgs::msg::Joy::L2] - joyMsg->joy_data[rover_msgs::msg::Joy::R2];

    // Initialize controls to previous values
    // =========================================================================
    armMsg.position[rover_msgs::msg::ArmCmd::JL] = _currentJLPos;
    armMsg.position[rover_msgs::msg::ArmCmd::J0] = _currentJ0Pos;
    armMsg.position[rover_msgs::msg::ArmCmd::J1] = _currentJ1Pos;
    armMsg.position[rover_msgs::msg::ArmCmd::J2] = _currentJ2Pos;
    armMsg.position[rover_msgs::msg::ArmCmd::GRIPPERTILT] = _currentGripperTilt;
    armMsg.position[rover_msgs::msg::ArmCmd::GRIPPERROT] = _currentGripperRot;

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

            _jacobian = computeJacobian(RobotState(_currentJLPos, _currentJ0Pos, _currentJ1Pos, _currentJ2Pos, _currentGripperTilt));
            _inverseJacobian = arma::inv(_jacobian);

            _computedJointVelocity = _inverseJacobian * _desiredCartesianVelocity;

            _constrainedJointVelocity = constrainJointVelocity(_computedJointVelocity, _maxJointVelocity);

            _currentJLPos += _constrainedJointVelocity(0);
            _currentJ0Pos += _constrainedJointVelocity(1);
            _currentJ1Pos += _constrainedJointVelocity(2);
            _currentJ2Pos += _constrainedJointVelocity(3);
            _currentGripperTilt += _constrainedJointVelocity(4);
        }

        if (_controlMode == JOINT)
        {
            if (_gripperMode)
            {
                _currentGripperTilt += _gripperTilt * MAX_JOINT_SPEED;
                _currentGripperRot += _gripperRot * MAX_JOINT_SPEED;
                armMsg.position[rover_msgs::msg::ArmCmd::GRIPPEROPENCLOSE] = _gripperState * MAX_JOINT_SPEED;
            }
            else
            {
                _currentJLPos += _posCmdJL * MAX_JOINT_SPEED;
                _currentJ0Pos += _posCmdJ0 * MAX_JOINT_SPEED * -1.0f;
                _currentJ1Pos += _posCmdJ1 * MAX_JOINT_SPEED;
                _currentJ2Pos += _posCmdJ2 * MAX_JOINT_SPEED;
            }
        }
    }

    _pub_arm_teleop_in->publish(armMsg);
}

arma::mat55 Teleop::computeJacobian(const RobotState &state)
{
    arma::mat55 J(arma::fill::zeros);

    // float q0 = state.q0;
    float q1 = state.q1;
    float q2 = state.q2;
    float q3 = state.q3;
    float q4 = state.q4;

    // float J0x = state.J0x;
    // float J0y = state.J0y;
    // float J0z = state.J0z;

    float J1x = state.J1x;
    float J1y = state.J1y;
    // float J1z = state.J1z;

    float J2x = state.J2x;
    float J2y = state.J2y;
    float J2z = state.J2z;

    float J3x = state.J3x;
    // float J3y = state.J3y;
    float J3z = state.J3z;

    float J4x = state.J4x;
    // float J4y = state.J4y;
    float J4z = state.J4z;

    // dot(Po.GetPosition(No), nx>) Result = J0x + J1x*cos(q1) + J2x*cos(q1)*cos(0.5*PI-q2) + J2z*cos(q1)*sin(0.5*PI-q2) + J3x*cos(q1)*cos(0.5*PI-q2-q3) + J3z*cos(q1)*sin(0.5*PI-q2-q3) - sin(q1)*(J1y+J2y) + J4x*sin(q1)*cos(0.5*PI-q2-q3-q4) + J4z*sin(q1)*sin(0.5*PI-q2-q3-q4)
    J(0, 0) = 0.0f;
    J(0, 1) = J1x * -sin(q1) + J2x * -sin(q1) * cos(0.5 * PI - q2) + J2z * -sin(q1) * sin(0.5 * PI - q2) + J3x * -sin(q1) * cos(0.5 * PI - q2 - q3) + J3z * -sin(q1) * sin(0.5 * PI - q2 - q3) - cos(q1) * (J1y + J2y) + J4x * cos(q1) * cos(0.5 * PI - q2 - q3 - q4) + J4z * cos(q1) * sin(0.5 * PI - q2 - q3 - q4);
    J(0, 2) = J2x * cos(q1) * -sin(0.5 * PI - q2) + J2z * cos(q1) * cos(0.5 * PI - q2) + J3x * cos(q1) * -sin(0.5 * PI - q2 - q3) + J3z * cos(q1) * cos(0.5 * PI - q2 - q3) + J4x * sin(q1) * -sin(0.5 * PI - q2 - q3 - q4) + J4z * sin(q1) * cos(0.5 * PI - q2 - q3 - q4);
    J(0, 3) = J3x * cos(q1) * -sin(0.5 * PI - q2 - q3) + J3z * cos(q1) * cos(0.5 * PI - q2 - q3) + J4x * sin(q1) * -sin(0.5 * PI - q2 - q3 - q4) + J4z * sin(q1) * cos(0.5 * PI - q2 - q3 - q4);
    J(0, 4) = J4x * sin(q1) * -sin(0.5 * PI - q2 - q3 - q4) + J4z * sin(q1) * cos(0.5 * PI - q2 - q3 - q4);

    // dot(Po.GetPosition(No), ny >) Result = J0y + q0 + J1x * sin(q1) + cos(q1) * (J1y + J2y) + J2x * sin(q1) * cos(0.5 * PI - q2) + J2z * sin(q1) * sin(0.5 * PI - q2) + J3x * sin(q1) * cos(0.5 * PI - q2 - q3) + J3z * sin(q1) * sin(0.5 * PI - q2 - q3) + J4x * sin(q1) * cos(0.5 * PI - q2 - q3 - q4) + J4z * sin(q1) * sin(0.5 * PI - q2 - q3 - q4);
    J(1, 0) = 1.0f;
    J(1, 1) = J1x * cos(q1) + -sin(q1) * (J1y + J2y) + J2x * cos(q1) * cos(0.5 * PI - q2) + J2z * cos(q1) * sin(0.5 * PI - q2) + J3x * cos(q1) * cos(0.5 * PI - q2 - q3) + J3z * cos(q1) * sin(0.5 * PI - q2 - q3) + J4x * cos(q1) * cos(0.5 * PI - q2 - q3 - q4) + J4z * cos(q1) * sin(0.5 * PI - q2 - q3 - q4);
    J(1, 2) = J2x * sin(q1) * -sin(0.5 * PI - q2) + J2z * sin(q1) * cos(0.5 * PI - q2) + J3x * sin(q1) * -sin(0.5 * PI - q2 - q3) + J3z * sin(q1) * cos(0.5 * PI - q2 - q3) + J4x * sin(q1) * -sin(0.5 * PI - q2 - q3 - q4) + J4z * sin(q1) * cos(0.5 * PI - q2 - q3 - q4);
    J(1, 3) = J3x * sin(q1) * -sin(0.5 * PI - q2 - q3) + J3z * sin(q1) * cos(0.5 * PI - q2 - q3) + J4x * sin(q1) * -sin(0.5 * PI - q2 - q3 - q4) + J4z * sin(q1) * cos(0.5 * PI - q2 - q3 - q4);
    J(1, 4) = J4x * sin(q1) * -sin(0.5 * PI - q2 - q3 - q4) + J4z * sin(q1) * cos(0.5 * PI - q2 - q3 - q4);

    // dot(Po.GetPosition(No), nz >) Result = J0z + J1z + J2z * cos(0.5 * PI - q2) + J3z * cos(0.5 * PI - q2 - q3) + J4z * cos(0.5 * PI - q2 - q3 - q4) - J2x * sin(0.5 * PI - q2) - J3x * sin(0.5 * PI - q2 - q3) - J4x * sin(0.5 * PI - q2 - q3 - q4);
    J(2, 0) = 0.0f;
    J(2, 1) = 0.0f;
    J(2, 2) = J2z * -sin(0.5 * PI - q2) + J3z * -sin(0.5 * PI - q2 - q3) + J4z * -sin(0.5 * PI - q2 - q3 - q4) - J2x * cos(0.5 * PI - q2) - J3x * cos(0.5 * PI - q2 - q3) - J4x * cos(0.5 * PI - q2 - q3 - q4);
    J(2, 3) = J3z * -sin(0.5 * PI - q2 - q3) + J4z * -sin(0.5 * PI - q2 - q3 - q4) - J3x * cos(0.5 * PI - q2 - q3) - J4x * cos(0.5 * PI - q2 - q3 - q4);
    J(2, 4) = J4z * -sin(0.5 * PI - q2 - q3 - q4) - J4x * cos(0.5 * PI - q2 - q3 - q4);

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
