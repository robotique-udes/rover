#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "rover_msgs/msg/joy.hpp"
#include "rover_msgs/msg/propulsion_motor.hpp"
#include "rover_msgs/msg/joy_demux_status.hpp"
#include "rover_msgs/msg/arm_cmd.hpp"
#include "rovus_lib/macros.h"
#include <armadillo>
#include <vector>

using namespace std::chrono_literals;

// #define MAX_JOINT_SPEED 0.0017453f
#define MAX_JOINT_SPEED 0.01f

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
    float _posCmdJL;
    float _posCmdJ0;
    float _posCmdJ1;
    float _posCmdJ2;

    float _gripperTilt;
    float _gripperRot;
    float _gripperState;

    bool _deadmanSwitch;
    bool _gripperMode;
    bool _cartesianMode;

    float _xVelocity;
    float _yVelocity;
    float _zVelocity;
    float _gripperTiltVelocity;
    float _gripperRotVelocity;

    float _currentJLPos;
    float _currentJ0Pos;
    float _currentJ1Pos;
    float _currentJ2Pos;
    float _currentGripperTilt;
    float _currentGripperRot;

    RobotState _currentState = RobotState(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    arma::vec5 _desiredCartesianVelocity;
    arma::mat55 _jacobian;
    arma::mat55 _jacobianInverse;
    arma::vec5 _jointVelocity;
    arma::vec5 _currentPos;
    arma::vec5 _newPos;
    arma::vec5 _desiredJointVelocities;
    arma::vec5 _currentCartesianPos;
    arma::vec5 _desiredCartesianPos;
    float _desiredGripperTilt;
    float _gimbleGripperTilt;
    float _gripperTiltWorld;

    arma::vec5 _desired;
    arma::vec5 _current;

    void joyCallback(const rover_msgs::msg::Joy::SharedPtr joyMsg);
    void cartesianCallback();
    void activateCartesian();
    arma::mat55 computeJacobian(const RobotState &state);
    arma::vec3 forwardKinematics(const RobotState &state);

    rclcpp::Subscription<rover_msgs::msg::Joy>::SharedPtr _sub_joy_arm;
    rclcpp::Publisher<rover_msgs::msg::ArmCmd>::SharedPtr _pub_arm_teleop_in;
    rover_msgs::msg::ArmCmd armMsg;

    rclcpp::TimerBase::SharedPtr _timer;

    const rover_msgs::msg::Joy::SharedPtr joyMsg;
};

Teleop::Teleop() : Node("teleop")
{
    _sub_joy_arm = this->create_subscription<rover_msgs::msg::Joy>(
        "/rover/arm/joy", 1, std::bind(&Teleop::joyCallback, this, std::placeholders::_1));
    _pub_arm_teleop_in = this->create_publisher<rover_msgs::msg::ArmCmd>("/rover/arm/cmd/in/teleop", 1);

    _cartesianMode = false;

    _timer = this->create_wall_timer(10ms, std::bind(&Teleop::cartesianCallback, this));
    _timer->cancel();
}

void Teleop::joyCallback(const rover_msgs::msg::Joy::SharedPtr joyMsg)
{
    _deadmanSwitch = joyMsg->joy_data[rover_msgs::msg::Joy::L1];
    _gripperMode = joyMsg->joy_data[rover_msgs::msg::Joy::A];

    _posCmdJL = joyMsg->joy_data[rover_msgs::msg::Joy::L2] - joyMsg->joy_data[rover_msgs::msg::Joy::R2];
    _posCmdJ0 = joyMsg->joy_data[rover_msgs::msg::Joy::JOYSTICK_RIGHT_SIDE];
    _posCmdJ1 = joyMsg->joy_data[rover_msgs::msg::Joy::JOYSTICK_LEFT_FRONT];
    _posCmdJ2 = joyMsg->joy_data[rover_msgs::msg::Joy::JOYSTICK_RIGHT_FRONT];

    _gripperTilt = joyMsg->joy_data[rover_msgs::msg::Joy::JOYSTICK_LEFT_FRONT] * -1.0f;
    _gripperRot = joyMsg->joy_data[rover_msgs::msg::Joy::JOYSTICK_RIGHT_SIDE];
    _gripperState = joyMsg->joy_data[rover_msgs::msg::Joy::L2] - joyMsg->joy_data[rover_msgs::msg::Joy::R2];

    _xVelocity = joyMsg->joy_data[rover_msgs::msg::Joy::JOYSTICK_LEFT_FRONT];
    _yVelocity = joyMsg->joy_data[rover_msgs::msg::Joy::JOYSTICK_LEFT_SIDE];
    _zVelocity = joyMsg->joy_data[rover_msgs::msg::Joy::JOYSTICK_RIGHT_FRONT];
    _gripperTiltVelocity = joyMsg->joy_data[rover_msgs::msg::Joy::L2];
    _gripperRotVelocity = joyMsg->joy_data[rover_msgs::msg::Joy::JOYSTICK_RIGHT_SIDE];

    armMsg.position[rover_msgs::msg::ArmCmd::JL] = _currentJLPos;
    armMsg.position[rover_msgs::msg::ArmCmd::J0] = _currentJ0Pos;
    armMsg.position[rover_msgs::msg::ArmCmd::J1] = _currentJ1Pos;
    armMsg.position[rover_msgs::msg::ArmCmd::J2] = _currentJ2Pos;
    armMsg.position[rover_msgs::msg::ArmCmd::GRIPPERTILT] = _currentGripperTilt;
    armMsg.position[rover_msgs::msg::ArmCmd::GRIPPERROT] = _currentGripperRot;

    // RCLCPP_INFO(rclcpp::get_logger("Dead Node"), "current q0: %f", armMsg.position[rover_msgs::msg::ArmCmd::JL]);
    // RCLCPP_INFO(rclcpp::get_logger("Dead Node"), "current q1: %f", _currentJ0Pos);
    // RCLCPP_INFO(rclcpp::get_logger("Dead Node"), "current q2: %f", _currentJ1Pos);
    // RCLCPP_INFO(rclcpp::get_logger("Dead Node"), "current q3: %f", _currentJ2Pos);
    // RCLCPP_INFO(rclcpp::get_logger("Dead Node"), "current q4: %f", _currentGripperTilt);
    // if (joyMsg->joy_data[rover_msgs::msg::Joy::CROSS_DOWN] == 1)
    // {
    //     _cartesianMode = !_cartesianMode;
    // }

    if (_deadmanSwitch)
    {
        if (joyMsg->joy_data[rover_msgs::msg::Joy::R1] == 1)
        {
            _desiredCartesianVelocity = {_xVelocity, _yVelocity * 0.0f, _zVelocity, _gripperTiltVelocity, _gripperRotVelocity *0.0f};

            // Base mode = x, y, and z

            // _desired = {_xVelocity, _zVelocity};
            // _current = {_currentJ1Pos, _currentJ2Pos};

            // WEIRD BEHAVIOR FOR DESIRED

            // RCLCPP_INFO(rclcpp::get_logger("Dead Node"), "desired x velocity: %f", _currentXZ(0));
            // RCLCPP_INFO(rclcpp::get_logger("Dead Node"), "desired y velocity: %f", _currentXZ(1));

            // RCLCPP_INFO(rclcpp::get_logger("DIK"), "\n_currentJLPos: %f\n_currentJ0Pos: %f\n_currentJ1Pos: %f\n_currentJ2Pos: %f\n_currentGripperTilt: %f", _currentJLPos, _currentJ0Pos, _currentJ1Pos, _currentJ2Pos, _currentGripperTilt);

            _jacobian = computeJacobian(RobotState(_currentJLPos, _currentJ0Pos, _currentJ1Pos, _currentJ2Pos, _currentGripperTilt));

            _jacobianInverse = arma::inv(_jacobian);

            _jointVelocity = _jacobianInverse * _desiredCartesianVelocity;

            // RCLCPP_INFO(rclcpp::get_logger("DIK"), "\nJLSpeed: %f\nJ0Speed: %f\nJ1Pos: %f\nJ2Pos: %f\nGripperTilt: %f", _jointVelocity(0), _jointVelocity(1), _jointVelocity(2), _jointVelocity(3), _jointVelocity(4));
            // RCLCPP_INFO(rclcpp::get_logger("Dead Node"), "Joint velocity q2 %f", _jointVelocityXZ(0));
            // RCLCPP_INFO(rclcpp::get_logger("Dead Node"), "Joint velocity q3 %f", _jointVelocityXZ(1));

            // Compute the incremental joint positions
            float deltaJL = _jointVelocity(0) * 0.001f;
            float deltaJ0 = _jointVelocity(1) * 0.001f;
            float deltaJ1 = _jointVelocity(2) * 0.001f;
            float deltaJ2 = _jointVelocity(3) * 0.001f;
            float deltaTilt = _jointVelocity(4) * 0.001f;
            // RCLCPP_INFO(rclcpp::get_logger("Dead Node"), "J1 %f", deltaJ1);
            // RCLCPP_INFO(rclcpp::get_logger("Dead Node"), "J2 %f", deltaJ2);

            // Update the gripper tilt to maintain the same world angle
            // _newPos(4) = _currentGripperTilt + deltaJ1 - deltaJ2;

            _currentJLPos += deltaJL;
            _currentJ0Pos += deltaJ0;
            _currentJ1Pos += deltaJ1;
            _currentJ2Pos += deltaJ2;
            _currentGripperTilt += deltaTilt;

            RCLCPP_INFO(rclcpp::get_logger("DIK"), "Combined Pos: %f",  _currentJ1Pos + _currentJ2Pos + _currentGripperTilt);

            // _currentPos = {_currentJLPos, _currentJ0Pos, _currentJ1Pos, _currentJ2Pos, _currentGripperTilt};

            // ALL INPUT CURRENT VERIFIED -- PROBLEM WITH INPUT Q1
            // _currentState.q0 = _currentJLPos;
            // _currentState.q1 = _currentJ0Pos;
            // _currentState.q2 = _currentJ1Pos;
            // _currentState.q3 = _currentJ2Pos;
            // _currentState.q4 = _currentGripperTilt;

            // RCLCPP_INFO(rclcpp::get_logger("Dead Node"), "current q0: %f", _currentState.q0);
            // RCLCPP_INFO(rclcpp::get_logger("Dead Node"), "current q1: %f", _currentState.q1);
            // RCLCPP_INFO(rclcpp::get_logger("Dead Node"), "current q2: %f", _currentState.q2);
            // RCLCPP_INFO(rclcpp::get_logger("Dead Node"), "current q3: %f", _currentState.q3);
            // RCLCPP_INFO(rclcpp::get_logger("Dead Node"), "current q4: %f", _currentState.q4);

            // _jacobian = computeJacobian(_currentState);

            // for (arma::uword i = 0; i < _jacobian.n_rows; ++i)
            // {
            // for (arma::uword j = 0; j < _jacobian.n_cols; ++j)
            // {
            // RCLCPP_INFO(rclcpp::get_logger("Dead Node"), "Jacobian[%lu][%lu]: %f", i, j, _jacobian(i, j));
            // }
            // }

            // _jacobianinverse = arma::inv(_jacobian);

            // for (arma::uword i = 0; i < _jacobianinverse.n_rows; ++i)
            // {
            // for (arma::uword j = 0; j < _jacobianinverse.n_cols; ++j)
            // {
            // RCLCPP_INFO(rclcpp::get_logger("Dead Node"), " Inverse Jacobian[%lu][%lu]: %f", i, j, _jacobianinverse(i, j));
            // }
            // }

            // _jointVelocity = _jacobianinverse * _desiredCartesianVelocity; //_desiredCartesianVelocity;
            //
            // for (arma::uword i = 0; i < _jointVelocity.n_elem; ++i)
            // {
            // RCLCPP_INFO(rclcpp::get_logger("Dead Node"), "Joint Velocity[%lu]: %f", i, _jointVelocity(i));
            // }

            // _newPos = _currentPos + (_jointVelocity);

            // RCLCPP_INFO(rclcpp::get_logger("Dead Node"), "new JL: %f", _newPos(0));
            // RCLCPP_INFO(rclcpp::get_logger("Dead Node"), "new J0: %f", _newPos(1));
            // RCLCPP_INFO(rclcpp::get_logger("Dead Node"), "new J1: %f", _newPos(2));
            // RCLCPP_INFO(rclcpp::get_logger("Dead Node"), "new J2: %f", _newPos(3));
            // RCLCPP_INFO(rclcpp::get_logger("Dead Node"), "new grip: %f", _newPos(4));

            // cartesianCallback();
            // arma::vec3 currentPos = forwardKinematics(_currentState);

            // arma::vec3 desiredPos = currentPos + arma::vec3({joyMsg->joy_data[rover_msgs::msg::Joy::JOYSTICK_LEFT_SIDE] * MAX_JOINT_SPEED,
            //                                                  joyMsg->joy_data[rover_msgs::msg::Joy::JOYSTICK_RIGHT_FRONT] * MAX_JOINT_SPEED,
            //                                                  joyMsg->joy_data[rover_msgs::msg::Joy::JOYSTICK_LEFT_FRONT] * MAX_JOINT_SPEED});

            // arma::mat J = computeJacobian(_currentState);

            // arma::vec3 posError = desiredPos - currentPos;

            // _desiredJointVelocities; //= arma::PInv(J) * posError;

            // _currentState.jl += _desiredJointVelocities(0) * MAX_JOINT_SPEED;
            // _currentState.j0 += _desiredJointVelocities(1) * MAX_JOINT_SPEED;
            // _currentState.j1 += _desiredJointVelocities(2) * MAX_JOINT_SPEED;
            // _currentState.j2 += _desiredJointVelocities(3) * MAX_JOINT_SPEED;
            // _currentState.gripperTilt += _desiredJointVelocities(4) * MAX_JOINT_SPEED;
        }
        else
        {
            if (_gripperMode)
            {
                RCLCPP_INFO(rclcpp::get_logger(""), "Here");
                _currentGripperTilt += _gripperTilt * MAX_JOINT_SPEED;
                _currentGripperRot += _gripperRot * MAX_JOINT_SPEED;
                armMsg.position[rover_msgs::msg::ArmCmd::GRIPPEROPENCLOSE] = _gripperState * MAX_JOINT_SPEED;
            }
            else
            {
                // _currentJLPos += _posCmdJL * MAX_JOINT_SPEED;
                // _currentJ0Pos += _posCmdJ0 * MAX_JOINT_SPEED * -1.0f;
                _currentJ1Pos += _posCmdJ1 * MAX_JOINT_SPEED;
                _currentJ2Pos += _posCmdJ2 * MAX_JOINT_SPEED;
            }
        }

        // armMsg.position[rover_msgs::msg::ArmCmd::JL] = _currentState.q0;
        // armMsg.position[rover_msgs::msg::ArmCmd::J0] = _currentState.q1;
        // armMsg.position[rover_msgs::msg::ArmCmd::J1] = _currentState.q2;
        // armMsg.position[rover_msgs::msg::ArmCmd::J2] = _currentState.q3;
        // armMsg.position[rover_msgs::msg::ArmCmd::GRIPPERTILT] = _currentState.q4;

        _pub_arm_teleop_in->publish(armMsg);
    }
}

void Teleop::cartesianCallback()
{
    // DESIRED VERIFIED
    // _desiredCartesianVelocity = {_xVelocity / 0.1, _yVelocity / 0.1, _zVelocity / 0.1, _gripperTiltVelocity / 0.1, _gripperRotVelocity / 0.1};

    // // RCLCPP_INFO(rclcpp::get_logger("Dead Node"), "current q0: %f", _currentJLPos);
    // // RCLCPP_INFO(rclcpp::get_logger("Dead Node"), "current q1: %f", _currentJ0Pos);
    // // RCLCPP_INFO(rclcpp::get_logger("Dead Node"), "current q2: %f", _currentJ1Pos);
    // // RCLCPP_INFO(rclcpp::get_logger("Dead Node"), "current q3: %f", _currentJ2Pos);
    // // RCLCPP_INFO(rclcpp::get_logger("Dead Node"), "current q4: %f", _currentGripperTilt);

    // _currentPos = {_currentJLPos, _currentJ0Pos, _currentJ1Pos, _currentJ2Pos, _currentGripperTilt};

    // // ALL INPUT CURRENT VERIFIED -- PROBLEM WITH INPUT Q1
    // _currentState.q0 = _currentJLPos;
    // _currentState.q1 = _currentJ0Pos;
    // _currentState.q2 = _currentJ1Pos;
    // _currentState.q3 = _currentJ2Pos;
    // _currentState.q4 = _currentGripperTilt;

    // // RCLCPP_INFO(rclcpp::get_logger("Dead Node"), "current q0: %f", _currentState.q0);
    // // RCLCPP_INFO(rclcpp::get_logger("Dead Node"), "current q1: %f", _currentState.q1);
    // // RCLCPP_INFO(rclcpp::get_logger("Dead Node"), "current q2: %f", _currentState.q2);
    // // RCLCPP_INFO(rclcpp::get_logger("Dead Node"), "current q3: %f", _currentState.q3);
    // // RCLCPP_INFO(rclcpp::get_logger("Dead Node"), "current q4: %f", _currentState.q4);

    // _jacobian = computeJacobian(_currentState);

    // _jointVelocity = pinv(_jacobian) * _desiredCartesianVelocity;

    // _newPos = _currentPos + (_jointVelocity / 0.1);

    // RCLCPP_INFO(rclcpp::get_logger("Dead Node"), "new JL: %f", _newPos(0));
    // RCLCPP_INFO(rclcpp::get_logger("Dead Node"), "new J0: %f", _newPos(1));
    // RCLCPP_INFO(rclcpp::get_logger("Dead Node"), "new J1: %f", _newPos(2));
    // RCLCPP_INFO(rclcpp::get_logger("Dead Node"), "new J2: %f", _newPos(3));
    // RCLCPP_INFO(rclcpp::get_logger("Dead Node"), "new grip: %f", _newPos(4));

    // _currentJLPos = _newPos(0) * MAX_JOINT_SPEED;
    // _currentJ0Pos = _newPos(1) * MAX_JOINT_SPEED;
    // _currentJ1Pos = _newPos(2) * MAX_JOINT_SPEED;
    // _currentJ2Pos = _newPos(3) * MAX_JOINT_SPEED;
    // _currentGripperTilt = _newPos(4) * MAX_JOINT_SPEED;

    // _currentState.gripperTilt += _desiredJointVelocities(4) * MAX_JOINT_SPEED;
    // arma::vec3 currentPos = forwardKinematics(_currentState);

    // arma::vec3 desiredPos = currentPos + arma::vec3({joyMsg->joy_data[rover_msgs::msg::Joy::JOYSTICK_LEFT_SIDE] * MAX_JOINT_SPEED,
    //                                                  joyMsg->joy_data[rover_msgs::msg::Joy::JOYSTICK_RIGHT_FRONT] * MAX_JOINT_SPEED,
    //                                                  joyMsg->joy_data[rover_msgs::msg::Joy::JOYSTICK_LEFT_FRONT] * MAX_JOINT_SPEED});

    // arma::mat J = computeJacobian(_currentState);

    // arma::vec3 posError = desiredPos - currentPos;

    // _desiredJointVelocities = arma::pinv(J) * posError;
}

arma::vec3 Teleop::forwardKinematics(const RobotState &state)
{
    // float x = state.J1x*cos(state.j0) + sin(state.j0)*(state.J1z + state.J2z) + sin(state.j0)*(state.J3z + state.J4z) +
    //           state.J2x*cos(state.j0) * cos(state.j1) + state.J3x * cos(state.j0) * cos(state.j1 + state.j2) +
    //           state.J4x * cos(state.j0) * cos(q2+q4) - J2y*sin(q2)*cos(q1) -
    // float y;
    // float z;

    // float gripperTilt;

    // float gripperRot;

    // return arma::vec3({x, y, z, gripperTilt, gripperRot});
}

arma::mat55 Teleop::computeJacobian(const RobotState &state)
{
    // float q2 = _currentXZ(0); // 1
    // float q3 = _currentXZ(1); // 1/2

    // float J0x = 0.0f, J0y = 0.0f, J0z = 0.0f;
    // float J1x = 0.0f, J1y = 0.0f, J1z = 0.0f;
    // float J2x = 0.65f, J2y = 0.0f, J2z = 0.0f;
    // float J3x = 0.625f, J3y = 0.0f, J3z = 0.0f;
    // float J4x = 0.217f, J4y = 0.0f, J4z = 0.0f;

    // J(0, 0) = J2x * -sin(0.5 * PI - q2) + J2z * cos(0.5 * PI - q2) + J3x * -sin(0.5 * PI - q2 - q3) + J3z * cos(0.5 * PI - q2 - q3) + J4x * -sin(0.5 * PI - q2 - q3) + J4z * cos(0.5 * PI - q2 - q3);
    // J(0, 1) = J3x * -sin(0.5 * PI - q2 - q3) + J3z * cos(0.5 * PI - q2 - q3) + J4x * -sin(0.5 * PI - q2 - q3) + J4z * cos(0.5 * PI - q2 - q3);
    //
    // J(1, 0) = J2z * -sin(0.5 * PI - q2) + J3z * -sin(0.5 * PI - q2 - q3) + J4z * -sin(0.5 * PI - q2 - q3) - J2x * cos(0.5 * PI - q2) - J3x * cos(0.5 * PI - q2 - q3) - J4x * cos(0.5 * PI - q2 - q3);
    // J(1, 1) = J3z * -sin(0.5 * PI - q2 - q3) + J4z * -sin(0.5 * PI - q2 - q3) - J3x * cos(0.5 * PI - q2 - q3) - J4x * cos(0.5 * PI - q2 - q3);

    // // PROMISSING
    // // NOT ENOUGH ENGAGEMENT OF J2 WHEN Q2 IS NEAR
    // J(0, 0) = J2x * -sin(q2) + J2z * cos(q2) + J3x * -sin(q2 - q3) + J3z * cos(q2 - q3) + J4x * -sin(q2 - q3) + J4z * cos(q2 - q3);
    // J(0, 1) = J3x * -sin(q2 - q3) + J3z * cos(q2 - q3) + J4x * -sin(q2 - q3) + J4z * cos(q2 - q3);

    // J(1, 0) = J2z * -sin(q2) + J3z * -sin(q2 - q3) + J4z * -sin(q2 - q3) - J2x * cos(q2) - J3x * cos(q2 - q3) - J4x * cos(q2 - q3);
    // J(1, 1) = J3z * -sin(q2 - q3) + J4z * -sin(q2 - q3) - J3x * cos(q2 - q3) - J4x * cos(q2 - q3);

    //  Result = J2x*cos(q2) + J2z*sin(q2) + J3x*cos(q3) + J3z*sin(q3)
    // J(0, 0) = J2z*-sin(q2);
    // J(0, 1) = J3z*-sin(q2 + q3);

    // ALSO PROMISSING
    // J(0, 0) = J2z*cos(q2) + J3z*cos(q2 + q3);
    // J(0, 1) = J3z*cos(q2 + q3) ;
    //
    // J(1, 0) = J2z*-sin(q2) - J3z*sin(q2 + q3);
    // J(1, 1) = J3z*-sin(q2);
    // // // Result = J2z*cos(q2) + J3z*cos(q3) - J2x*sin(q2) - J3x*sin(q3)
    // J(1, 0) = J2z*cos(q2);
    // J(1, 1) = J3z*cos(q2 + q3);

    float q0 = state.q0;
    float q1 = state.q1;
    float q2 = state.q2;
    float q3 = state.q3;
    float q4 = state.q4;

    float J0x = state.J0x;
    float J0y = state.J0y;
    float J0z = state.J0z;

    float J1x = state.J1x;
    float J1y = state.J1y;
    float J1z = state.J1z;

    float J2x = state.J2x;
    float J2y = state.J2y;
    float J2z = state.J2z;

    float J3x = state.J3x;
    float J3y = state.J3y;
    float J3z = state.J3z;

    float J4x = state.J4x;
    float J4y = state.J4y;
    float J4z = state.J4z;

    arma::mat55 J(arma::fill::zeros);

    // J(0, 0) = 0.0f;
    // J(0, 1) = J1x * -sin(q1) + J2x * -sin(q1) * cos(0.5 * PI - q2) + J2z * -sin(q1) * sin(0.5 * PI - q2) + J3x * -sin(q1) * cos(0.5 * PI - q2 - q3) + J3z * -sin(q1) * sin(0.5 * PI - q2 - q3) + J4x * -sin(q1) * cos(0.5 * PI - q2 - q3) + J4z * -sin(q1) * sin(0.5 * PI - q2 - q3) - cos(q1) * (J1y + J2y) - cos(q1) * (J3y + J4y);
    // J(0, 2) = J2x * cos(q1) * -sin(0.5 * PI - q2) + J2z * cos(q1) * cos(0.5 * PI - q2) + J3x * cos(q1) * -sin(0.5 * PI - q2 - q3) + J3z * cos(q1) * cos(0.5 * PI - q2 - q3) + J4x * cos(q1) * -sin(0.5 * PI - q2 - q3) + J4z * cos(q1) * cos(0.5 * PI - q2 - q3);
    // J(0, 3) = J3x * cos(q1) * -sin(0.5 * PI - q2 - q3) + J3z * cos(q1) * cos(0.5 * PI - q2 - q3) + J4x * cos(q1) * -sin(0.5 * PI - q2 - q3) + J4z * cos(q1) * cos(0.5 * PI - q2 - q3);
    // J(0, 4) = J4x * cos(q1) * cos(0.5 * PI - q2 - q3) + J4z * cos(q1) * sin(0.5 * PI - q2 - q3);

    J(0, 0) = 0.0f;
    J(0, 1) = J1x * -sin(q1) + J2x * -sin(q1) * cos(0.5 * PI - q2) + J2z * -sin(q1) * sin(0.5 * PI - q2) + J3x * -sin(q1) * cos(0.5 * PI - q2 - q3) + J3z * -sin(q1) * sin(0.5 * PI - q2 - q3) + J4x * -sin(q1) * cos(0.5 * PI - q2 - q3) + J4z * -sin(q1) * sin(0.5 * PI - q2 - q3) - cos(q1) * (J1y + J2y) - cos(q1) * (J3y + J4y);
    J(0, 2) = J2x * cos(q1) * -sin(0.5 * PI - q2) + J2z * cos(q1) * cos(0.5 * PI - q2) + J3x * cos(q1) * -sin(0.5 * PI - q2 - q3) + J3z * cos(q1) * cos(0.5 * PI - q2 - q3) + J4x * cos(q1) * -sin(0.5 * PI - q2 - q3) + J4z * cos(q1) * cos(0.5 * PI - q2 - q3);
    J(0, 3) = J3x * cos(q1) * -sin(0.5 * PI - q2 - q3) + J3z * cos(q1) * cos(0.5 * PI - q2 - q3) + J4x * cos(q1) * -sin(0.5 * PI - q2 - q3) + J4z * cos(q1) * cos(0.5 * PI - q2 - q3);
    J(0, 4) = J4x * cos(q1) * cos(0.5 * PI - q2 - q3) + J4z * cos(q1) * sin(0.5 * PI - q2 - q3);

    J(1, 0) = 1.0;
    J(1, 1) = J1x * cos(q1) + -sin(q1) * (J1y + J2y) + -sin(q1) * (J3y + J4y) + J2x * cos(q1) * cos(0.5 * PI - q2) + J2z * cos(q1) * sin(0.5 * PI - q2) + J3x * cos(q1) * cos(0.5 * PI - q2 - q3) + J3z * cos(q1) * sin(0.5 * PI - q2 - q3) + J4x * cos(q1) * cos(0.5 * PI - q2 - q3) + J4z * cos(q1) * sin(0.5 * PI - q2 - q3);
    J(1, 2) = J2x * sin(q1) * -sin(0.5 * PI - q2) + J2z * sin(q1) * cos(0.5 * PI - q2) + J3x * sin(q1) * -sin(0.5 * PI - q2 - q3) + J3z * sin(q1) * cos(0.5 * PI - q2 - q3) + J4x * sin(q1) * -sin(0.5 * PI - q2 - q3) + J4z * sin(q1) * cos(0.5 * PI - q2 - q3);
    J(1, 3) = J3x * sin(q1) * -sin(0.5 * PI - q2 - q3) + J3z * sin(q1) * cos(0.5 * PI - q2 - q3) + J4x * sin(q1) * -sin(0.5 * PI - q2 - q3) + J4z * sin(q1) * cos(0.5 * PI - q2 - q3);
    J(1, 4) = J4x * sin(q1) * -sin(0.5 * PI - q2 - q3) + J4z * cos(q1) * sin(0.5 * PI - q2 - q3);

    J(2, 0) = 0.0;
    J(2, 1) = 0.0;
    J(2, 2) = J2z * -sin(0.5 * PI - q2) + J3z * -sin(0.5 * PI - q2 - q3) + J4z * -sin(0.5 * PI - q2 - q3) - J2x * cos(0.5 * PI - q2) - J3x * cos(0.5 * PI - q2 - q3) - J4x * cos(0.5 * PI - q2 - q3);
    J(2, 3) = J3z * -sin(0.5 * PI - q2 - q3) + J4z * -sin(0.5 * PI - q2 - q3) - J3x * cos(0.5 * PI - q2 - q3) - J4x * cos(0.5 * PI - q2 - q3);
    J(2, 4) = J4z * -sin(0.5 * PI - q2 - q3) - J4x * sin(0.5 * PI - q2 - q3);

    J(3, 0) = 0.0f;
    J(3, 1) = 0.0f;
    J(3, 2) = 1.0f;
    J(3, 3) = 1.0f;
    J(3, 4) = -1.0f;

    J(4, 0) = 0.0f;
    J(4, 1) = 1.0f;
    J(4, 2) = 0.0f;
    J(4, 3) = 0.0f;
    J(4, 4) = 0.0f;

    return J;
}

void Teleop::activateCartesian()
{
    if (!_cartesianMode)
    {
        _cartesianMode = true;
        _timer->reset();
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto teleop_node = std::make_shared<Teleop>();
    rclcpp::spin(teleop_node);
    rclcpp::shutdown();
    return 0;
}
