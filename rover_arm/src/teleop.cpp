// #include <Eigen/LU>
// #include <Eigen/Core>

#include <armadillo>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "rover_msgs/msg/joy.hpp"
#include "rover_msgs/msg/propulsion_motor.hpp"
#include "rover_msgs/msg/joy_demux_status.hpp"
#include "rover_msgs/msg/arm_cmd.hpp"
#include "rovus_lib/macros.h"

#define MAX_JOINT_SPEED 0.0017453f // 10 deg/s - (0.1745 rad/s)

// PUT IN ENUM
#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2
#define ALPHA 3
#define PHI 4

class Teleop : public rclcpp::Node
{
public:
    Teleop();

private:
    // Private members
    //  =========================================================================
    enum class eControlMode : uint8_t
    {
        JOINT,
        CARTESIAN
    };

    float _J1x = 1;
    float _J1y = 74;
    float _J1z = 1;

    float _J2x = 1;
    float _J2y = 650;
    float _J2z = 1;

    float _J3x = 1;
    float _J3y = 625;
    float _J3z = 1;

    float _J4x = 1;
    float _J4y = 209.5;
    float _J4z = 1;

    float _posCmdJL;
    float _posCmdJ0;
    float _posCmdJ1;
    float _posCmdJ2;
    float _gripperTilt;
    float _gripperRot;
    float _gripperOpen;
    float _gripperClose;

    float _deadmanSwitch;
    float _gripperMode;
    float _cartesian;

    float _speedControl = MAX_JOINT_SPEED; // 10 deg/s - (0.1745 rad/s)

    // Set Constrain values
    //  =========================================================================
    float _currentJLPos;
    float _currentJ0Pos;
    float _currentJ1Pos;
    float _currentJ2Pos;
    float _currentGripperTilt;
    float _currentGripperRot;
    float _currentGripperOpenClose;

    float _desiredXPos;
    float _desiredYPos;
    float _desiredZPos;

    // Set varying values for inverse kin (TO BE PUT IN YAML FILE)
    //  =========================================================================
    float _posCmdX;
    float _posCmdY;
    float _posCmdZ;

    // Vector definition (CHANGE AXIS / modify motionGen)
    // =========================================================================
    // float _rx = _currentJ0Pos + _link1Length * sin(_currentJ1Pos) * sin(_currentJ2Pos) + _link2Length * sin(_currentJ1Pos) * sin(_currentJ2Pos + _currentGripperTilt) + _link3Length * sin(_currentJ1Pos) * sin(_currentJ2Pos + _currentGripperTilt + _q5);
    // float _ry = -1.0f * cos(_currentJ2Pos) * (_link1Length * sin(_currentJ2Pos) + _link2Length * sin(_currentJ2Pos + _currentGripperTilt)) + _link3Length * sin(_currentJ2Pos + _currentGripperTilt + _q5);
    // float _rz = _link1Length * cos(_currentJ2Pos) + _link2Length * cos(_currentJ2Pos + _currentGripperTilt) + _link3Length * cos(_currentJ2Pos + _currentGripperTilt + _q5);

    // float alpha_ = _currentJ1Pos + _currentJ2Pos + _currentGripperTilt;
    // float phi_ = _currentJ0Pos;

    // Matrix and vector init
    // =========================================================================
    arma::vec _desiredEndEffectorPos;
    arma::vec _currentEndEffectorPos;
    arma::vec _posError;
    arma::mat _jacobian;
    arma::vec _jointVelocities;

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
    _desiredEndEffectorPos.set_size(5);
    _currentEndEffectorPos.set_size(5);
    _posError.set_size(5);
    _jacobian.set_size(5, 5);
    _jointVelocities.set_size(5);

    // Mode selection
    // =========================================================================
    _deadmanSwitch = joyMsg->joy_data[rover_msgs::msg::Joy::L1];
    _gripperMode = joyMsg->joy_data[rover_msgs::msg::Joy::R1];
    _cartesian = joyMsg->joy_data[rover_msgs::msg::Joy::A];
    // JOINT controls
    // =========================================================================
    _posCmdJL = joyMsg->joy_data[rover_msgs::msg::Joy::JOYSTICK_LEFT_SIDE];
    _posCmdJ0 = joyMsg->joy_data[rover_msgs::msg::Joy::JOYSTICK_RIGHT_SIDE];
    _posCmdJ1 = joyMsg->joy_data[rover_msgs::msg::Joy::JOYSTICK_LEFT_FRONT];
    _posCmdJ2 = joyMsg->joy_data[rover_msgs::msg::Joy::JOYSTICK_RIGHT_FRONT];

    // CARTESIAN controls
    // =========================================================================
    _posCmdX = joyMsg->joy_data[rover_msgs::msg::Joy::JOYSTICK_LEFT_FRONT];
    _posCmdY = joyMsg->joy_data[rover_msgs::msg::Joy::JOYSTICK_LEFT_SIDE];
    _posCmdZ = joyMsg->joy_data[rover_msgs::msg::Joy::JOYSTICK_RIGHT_FRONT];

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
    armMsg.position[rover_msgs::msg::ArmCmd::GRIPPEROPENCLOSE] = _currentGripperOpenClose;

    // Control logic
    // =========================================================================
    if (_deadmanSwitch)
    {
        if (!_cartesian) // TODO Conditional for joint
        {
            if (_gripperMode)
            {
                _currentGripperTilt += _gripperRot * _speedControl;
                _currentGripperRot += _gripperTilt * _speedControl;
                _currentGripperOpenClose += (_gripperOpen - _gripperClose) * _speedControl;
            }
            else
            {
                _currentJLPos += _posCmdJL * _speedControl;
                _currentJ0Pos += _posCmdJ0 * _speedControl * -1.0f;
                _currentJ1Pos += _posCmdJ1 * _speedControl;
                _currentJ2Pos += _posCmdJ2 * _speedControl;
            }
        }
        else if (_cartesian) // TODO Conditional for cartesian
        {
            // Current positions
            // =========================================================================
            // Update the end-effector position using the new kinematic representation

            _currentEndEffectorPos(X_AXIS) = _J1x * cos(_currentJ0Pos) + sin(_currentJ0Pos) * (_J1z + _J2z) +
                                             sin(_currentJ0Pos) * (_J3z + _J4z) +
                                             _J2x * cos(_currentJ0Pos) * cos(_currentJ1Pos) +
                                             _J3x * cos(_currentJ0Pos) * cos(_currentJ1Pos + _currentJ2Pos) +
                                             _J4x * cos(_currentJ0Pos) * cos(_currentJ1Pos + _currentGripperTilt) -
                                             _J2y * sin(_currentJ1Pos) * cos(_currentJ0Pos) -
                                             _J3y * cos(_currentJ0Pos) * sin(_currentJ1Pos + _currentJ2Pos) -
                                             _J4y * cos(_currentJ0Pos) * sin(_currentJ1Pos + _currentGripperTilt);

            _currentEndEffectorPos(Y_AXIS) = _J1y +
                                             _J2x * sin(_currentJ1Pos) +
                                             _J2y * cos(_currentJ1Pos) +
                                             _J3x * sin(_currentJ1Pos + _currentJ2Pos) +
                                             _J3y * cos(_currentJ1Pos + _currentJ2Pos) +
                                             _J4x * sin(_currentJ1Pos + _currentGripperTilt) +
                                             _J4y * cos(_currentJ1Pos + _currentGripperTilt);

            _currentEndEffectorPos(Z_AXIS) = cos(_currentJ0Pos) * (_J1z + _J2z) +
                                             cos(_currentJ0Pos) * (_J3z + _J4z) +
                                             _J2y * sin(_currentJ0Pos) * sin(_currentJ1Pos) +
                                             _J3y * sin(_currentJ0Pos) * sin(_currentJ1Pos + _currentJ2Pos) +
                                             _J4y * sin(_currentJ0Pos) * sin(_currentJ1Pos + _currentGripperTilt) -
                                             _J1x * sin(_currentJ0Pos) -
                                             _J2x * sin(_currentJ0Pos) * cos(_currentJ1Pos) -
                                             _J3x * sin(_currentJ0Pos) * cos(_currentJ1Pos + _currentJ2Pos) -
                                             _J4x * sin(_currentJ0Pos) * cos(_currentJ1Pos + _currentGripperTilt);

            // Desired positions (JOINTS)
            // =========================================================================
            _desiredEndEffectorPos(X_AXIS) = _currentEndEffectorPos(X_AXIS) + _posCmdX * _speedControl;
            _desiredEndEffectorPos(Y_AXIS) = _currentEndEffectorPos(Y_AXIS) + _posCmdY * _speedControl;
            _desiredEndEffectorPos(Z_AXIS) = _currentEndEffectorPos(Z_AXIS) + _posCmdZ * _speedControl;

            // _desiredJLPos = _currentJLPos + (_posCmdJL * _speedControl);
            // _desiredJ0Pos = _currentJ0Pos + (_posCmdJ0 * _speedControl * -1.0f);
            // _desiredJ1Pos = _currentJ1Pos + (_posCmdJ1 * _speedControl);
            // _desiredJ2Pos = _currentJ2Pos + (_currentJ2Pos += _posCmdJ2 * _speedControl);
            // _desiredGripperTilt = _currentGripperTilt + ((_gripperOpen - _gripperClose) * _speedControl);

            // // Desired position (CARTESIAN)
            // // =========================================================================
            // _desiredEndEffectorPos(X_AXIS) = _desiredJLPos +
            //                                  _link1Length * sin(_desiredJ0Pos) * sin(_desiredJ1Pos) +
            //                                  _link2Length * sin(_desiredJ0Pos) * sin(_desiredJ1Pos + _desiredJ2Pos) +
            //                                  _link3Length * sin(_desiredJ0Pos) * sin(_desiredJ1Pos + _desiredJ2Pos + _desiredGripperTilt);
            // _desiredEndEffectorPos(Y_AXIS) = -1.0f * cos(_desiredJ1Pos) * (_link1Length * sin(_desiredJ1Pos) + _link2Length * sin(_desiredJ1Pos + _desiredJ2Pos)) +
            //                                  _link3Length * sin(_desiredJ1Pos + _desiredJ2Pos + _desiredGripperTilt);
            // _desiredEndEffectorPos(Z_AXIS) = _link1Length * cos(_desiredJ1Pos) +
            //                                  _link2Length * cos(_desiredJ1Pos + _desiredJ2Pos) +
            //                                  _link3Length * cos(_desiredJ1Pos + _desiredJ2Pos + _desiredGripperTilt);

            _posError = _desiredEndEffectorPos - _currentEndEffectorPos;

            // Update the Jacobian matrix using the new kinematic representation

            _jacobian(X_AXIS, 0) = 0;
            _jacobian(X_AXIS, 1) = -_J1x * sin(_currentJ0Pos) + cos(_currentJ0Pos) * (_J1z + _J2z) + cos(_currentJ0Pos) * (_J3z + _J4z) +
                                   _J2x * cos(_currentJ0Pos) * cos(_currentJ1Pos) - _J2y * sin(_currentJ1Pos) * sin(_currentJ0Pos) +
                                   _J3x * cos(_currentJ0Pos) * cos(_currentJ1Pos + _currentJ2Pos) - _J3y * sin(_currentJ1Pos + _currentJ2Pos) * sin(_currentJ0Pos) +
                                   _J4x * cos(_currentJ0Pos) * cos(_currentJ1Pos + _currentGripperTilt) - _J4y * sin(_currentJ1Pos + _currentGripperTilt) * sin(_currentJ0Pos);
            _jacobian(X_AXIS, 2) = -_J2x * sin(_currentJ0Pos) * sin(_currentJ1Pos) + _J2y * cos(_currentJ1Pos) * cos(_currentJ0Pos) +
                                   -_J3x * sin(_currentJ0Pos) * sin(_currentJ1Pos + _currentJ2Pos) + _J3y * cos(_currentJ1Pos + _currentJ2Pos) * cos(_currentJ0Pos) +
                                   -_J4x * sin(_currentJ0Pos) * sin(_currentJ1Pos + _currentGripperTilt) + _J4y * cos(_currentJ1Pos + _currentGripperTilt) * cos(_currentJ0Pos);
            _jacobian(X_AXIS, 3) = -_J3x * sin(_currentJ0Pos) * sin(_currentJ1Pos + _currentJ2Pos) + _J3y * cos(_currentJ1Pos + _currentJ2Pos) * cos(_currentJ0Pos) +
                                   -_J4x * sin(_currentJ0Pos) * sin(_currentJ1Pos + _currentGripperTilt) + _J4y * cos(_currentJ1Pos + _currentGripperTilt) * cos(_currentJ0Pos);
            _jacobian(X_AXIS, 4) = -_J4x * sin(_currentJ0Pos) * sin(_currentJ1Pos + _currentGripperTilt) + _J4y * cos(_currentJ1Pos + _currentGripperTilt) * cos(_currentJ0Pos);

            _jacobian(Y_AXIS, 0) = 0;
            _jacobian(Y_AXIS, 1) = 0;
            _jacobian(Y_AXIS, 2) = _J2x * cos(_currentJ1Pos) - _J2y * sin(_currentJ1Pos) +
                                   _J3x * cos(_currentJ1Pos + _currentJ2Pos) - _J3y * sin(_currentJ1Pos + _currentJ2Pos) +
                                   _J4x * cos(_currentJ1Pos + _currentGripperTilt) - _J4y * sin(_currentJ1Pos + _currentGripperTilt);
            _jacobian(Y_AXIS, 3) = _J3x * cos(_currentJ1Pos + _currentJ2Pos) - _J3y * sin(_currentJ1Pos + _currentJ2Pos) +
                                   _J4x * cos(_currentJ1Pos + _currentGripperTilt) - _J4y * sin(_currentJ1Pos + _currentGripperTilt);
            _jacobian(Y_AXIS, 4) = _J4x * cos(_currentJ1Pos + _currentGripperTilt) - _J4y * sin(_currentJ1Pos + _currentGripperTilt);

            _jacobian(Z_AXIS, 0) = 0;
            _jacobian(Z_AXIS, 1) = 0;
            _jacobian(Z_AXIS, 2) = _J2y * cos(_currentJ0Pos) * sin(_currentJ1Pos) - _J2x * sin(_currentJ0Pos) * cos(_currentJ1Pos) -
                                   _J3x * sin(_currentJ0Pos) * cos(_currentJ1Pos + _currentJ2Pos) + _J3y * cos(_currentJ0Pos) * sin(_currentJ1Pos + _currentJ2Pos) -
                                   _J4x * sin(_currentJ0Pos) * cos(_currentJ1Pos + _currentGripperTilt) + _J4y * cos(_currentJ0Pos) * sin(_currentJ1Pos + _currentGripperTilt);
            _jacobian(Z_AXIS, 3) = _J3y * cos(_currentJ0Pos) * sin(_currentJ1Pos + _currentJ2Pos) - _J3x * sin(_currentJ0Pos) * cos(_currentJ1Pos + _currentJ2Pos) -
                                   _J4x * sin(_currentJ0Pos) * cos(_currentJ1Pos + _currentGripperTilt) + _J4y * cos(_currentJ0Pos) * sin(_currentJ1Pos + _currentGripperTilt);
            _jacobian(Z_AXIS, 4) = _J4y * cos(_currentJ0Pos) * sin(_currentJ1Pos + _currentGripperTilt) - _J4x * sin(_currentJ0Pos) * cos(_currentJ1Pos + _currentGripperTilt);

            _jacobian(ALPHA, 0) = 0;
            _jacobian(ALPHA, 1) = 1;
            _jacobian(ALPHA, 2) = 1;
            _jacobian(ALPHA, 3) = 1;
            _jacobian(ALPHA, 4) = 0;

            _jacobian(PHI, 0) = 0;
            _jacobian(PHI, 1) = 1;
            _jacobian(PHI, 2) = 0;
            _jacobian(PHI, 3) = 0;
            _jacobian(PHI, 4) = 0;

            _jointVelocities = arma::pinv(_jacobian) * _posError;

            _currentJLPos += _jointVelocities(0);
            _currentJ0Pos += _jointVelocities(1);
            _currentJ1Pos += _jointVelocities(2);
            _currentJ2Pos += _jointVelocities(3);
            _currentGripperTilt += _jointVelocities(4);
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
