#include <armadillo>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "rover_msgs/msg/joy.hpp"
#include "rover_msgs/msg/propulsion_motor.hpp"
#include "rover_msgs/msg/joy_demux_status.hpp"
#include "rover_msgs/msg/arm_msg.hpp"
#include "rovus_lib/macros.h"
#include "rovus_lib/timer.hpp"

#include "keybinding.hpp"
#include "robot_configuration.hpp"

// =============================================================================
// This node has for goal to control the robotic arm of the rover.
//
// For broader information about the mathematics behind the calculations, please
// consult the readme
//
// This node sends its calculated messages to the motors and to the simulation
// =============================================================================

constexpr uint64_t DEBOUNCE_TIME_MS = 500ul;
constexpr float JOINT_CONTROL_SPEED_FACTOR = 0.5f; // Factor of max speed

bool isPressed(float buttonValue_);

class Teleop : public rclcpp::Node
{
public:
    enum class eJointIndex : uint8_t
    {
        JL = rover_msgs::msg::ArmMsg::JL,
        J0 = rover_msgs::msg::ArmMsg::J0,
        J1 = rover_msgs::msg::ArmMsg::J1,
        J2 = rover_msgs::msg::ArmMsg::J2,
        GRIPPER_TILT = rover_msgs::msg::ArmMsg::GRIPPER_TILT,
        GRIPPER_ROT = rover_msgs::msg::ArmMsg::GRIPPER_ROT,
        GRIPPER_CLOSE = rover_msgs::msg::ArmMsg::GRIPPER_CLOSE,
        eLAST
    };

    enum class eControlMode : uint8_t
    {
        JOINT = 0,
        CARTESIAN = 1
    };

    Teleop();

private:
    rclcpp::Subscription<rover_msgs::msg::Joy>::SharedPtr _sub_joyArm;
    rclcpp::Subscription<rover_msgs::msg::ArmMsg>::SharedPtr _sub_armPosition;
    rclcpp::Publisher<rover_msgs::msg::ArmMsg>::SharedPtr _pub_armCmd;
    rclcpp::TimerBase::SharedPtr _timer_armHeartbeat;

    eJointIndex _selectedJoint = eJointIndex::J0;
    float _gripperPos = -1.0f;
    // // CARTESIAN VECTORS AND MATRICES
    // //  =========================================================================
    // arma::vec5 _desiredCartesianVelocity;
    // arma::mat55 _jacobian;
    // arma::mat55 _inverseJacobian;
    // arma::vec5 _computedJointVelocity;
    // arma::vec5 _constrainedJointVelocity;
    // arma::vec _maxJointVelocity;
    // arma::vec5 _cartesianMaxJointVelocity;
    // arma::mat _pointPositions;

#warning TODO: Timer to make sure it s "alive"
    bool _currentPosInvalid = true;
    float _currentJointsPos[(uint8_t)eJointIndex::eLAST] = {0};
    Teleop::eControlMode _controlMode = Teleop::eControlMode::JOINT;
    Timer<uint64_t, millis> timerDebounce = Timer<uint64_t, millis>(DEBOUNCE_TIME_MS);

    // Private methods
    //  =========================================================================
    void CB_joy(const rover_msgs::msg::Joy::SharedPtr joyMsg);
    void CB_currentPos(const rover_msgs::msg::ArmMsg::SharedPtr armCurrentPos);
    void CB_watchdog(bool *lostHB_);
    // arma::mat55 computeJacobian(const sJoint &state);
    // arma::mat computeDirectKinematic(const sJoint &state);
    arma::vec5 constrainJointVelocity(arma::vec5 jointVelocity, arma::vec maxJointVelocity);
    rover_msgs::msg::ArmMsg getZeroMsg(void);
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Teleop>());
    rclcpp::shutdown();
}

Teleop::Teleop() : Node("teleop")
{
    _timer_armHeartbeat = this->create_wall_timer(std::chrono::milliseconds(500), [this]()
                                                  { this->CB_watchdog(&_currentPosInvalid); });

    _sub_joyArm = this->create_subscription<rover_msgs::msg::Joy>("/rover/arm/joy",
                                                                  1,
                                                                  std::bind(&Teleop::CB_joy, this, std::placeholders::_1));

    _sub_armPosition = this->create_subscription<rover_msgs::msg::ArmMsg>("/rover/arm/status/current_positions",
                                                                          1,
                                                                          [this](const rover_msgs::msg::ArmMsg::SharedPtr msg)
                                                                          { this->CB_currentPos(msg); });

    _pub_armCmd = this->create_publisher<rover_msgs::msg::ArmMsg>("/rover/arm/cmd/goal_pos", 1);

    _currentJointsPos[(uint8_t)eJointIndex::GRIPPER_CLOSE] = 1.0f;
}

void Teleop::CB_joy(const rover_msgs::msg::Joy::SharedPtr joyMsg)
{
    if (!isPressed(joyMsg->joy_data[rover_msgs::msg::Joy::L1])) // deadman switch
    {
        _pub_armCmd->publish(this->getZeroMsg());
        return;
    }

    if (_currentPosInvalid)
    {
        _pub_armCmd->publish(this->getZeroMsg());
        return;
    }

    // if (!IN_ERROR(joyMsg->joy_data[rover_msgs::msg::Joy::Y], 0.01, 0.0))
    // {
    //     _controlMode = _controlMode == eControlMode::JOINT ? eControlMode::CARTESIAN : eControlMode::JOINT;
    // }

    float _goalJointsPos[(uint8_t)eJointIndex::eLAST] = {0};

    if (_controlMode == eControlMode::JOINT)
    {
        for (uint8_t i = 0; i < (uint8_t)eJointIndex::eLAST; i++)
        {
            _goalJointsPos[i] = _currentJointsPos[i];
        }

        if (isPressed(joyMsg->joy_data[KEYBINDING::JOINT_SELECT_INC])) // selected joint++
        {
            _selectedJoint = (eJointIndex)((uint8_t)_selectedJoint + 1);
            if (_selectedJoint == eJointIndex::eLAST)
            {
                _selectedJoint = (eJointIndex)0;
            }
        }
        else if (isPressed(joyMsg->joy_data[KEYBINDING::JOINT_SELECT_DEC])) // selected joint--
        {
            if (_selectedJoint == eJointIndex(0))
            {
                _selectedJoint = (eJointIndex)((uint8_t)eJointIndex::eLAST - 1);
            }
            else
            {
                _selectedJoint = (eJointIndex)((uint8_t)_selectedJoint - 1);
            }
        }

        // CMD JL
        if (isPressed(joyMsg->joy_data[KEYBINDING::JL_FWD]))
        {
            _goalJointsPos[(uint8_t)eJointIndex::JL] =
                _currentJointsPos[(uint8_t)eJointIndex::JL] + MAX_VELOCITY_JL;
        }
        else if (isPressed(joyMsg->joy_data[KEYBINDING::JL_REV]))
        {
            _goalJointsPos[(uint8_t)eJointIndex::JL] =
                _currentJointsPos[(uint8_t)eJointIndex::JL] - MAX_VELOCITY_JL;
        }

        // CMD J0
        if (isPressed(joyMsg->joy_data[KEYBINDING::J0_FWD]))
        {
            _goalJointsPos[(uint8_t)eJointIndex::J0] =
                _currentJointsPos[(uint8_t)eJointIndex::J0] + MAX_VELOCITY_J0;
        }
        else if (isPressed(joyMsg->joy_data[KEYBINDING::J0_REV]))
        {
            _goalJointsPos[(uint8_t)eJointIndex::J0] =
                _currentJointsPos[(uint8_t)eJointIndex::J0] - MAX_VELOCITY_J0;
        }

        // CMD J1
        _goalJointsPos[(uint8_t)eJointIndex::J1] =
            (joyMsg->joy_data[KEYBINDING::J1] * MAX_VELOCITY_J1);
        // CMD J2
        _goalJointsPos[(uint8_t)eJointIndex::J2] =
            _currentJointsPos[(uint8_t)eJointIndex::J2] + (joyMsg->joy_data[KEYBINDING::J2] * MAX_VELOCITY_J2);

        // CMD GRIP_TILT
        if (isPressed(joyMsg->joy_data[KEYBINDING::GRIPPER_TILT_FWD]))
        {
            _goalJointsPos[(uint8_t)eJointIndex::GRIPPER_TILT] =
                _currentJointsPos[(uint8_t)eJointIndex::GRIPPER_TILT] + MAX_VELOCITY_GRIPPER_TILT;
        }
        else if (isPressed(joyMsg->joy_data[KEYBINDING::GRIPPER_TILT_REV]))
        {
            _goalJointsPos[(uint8_t)eJointIndex::GRIPPER_TILT] =
                _currentJointsPos[(uint8_t)eJointIndex::GRIPPER_TILT] - MAX_VELOCITY_GRIPPER_TILT;
        }

        // CMD GRIP_ROT
        if (isPressed(joyMsg->joy_data[KEYBINDING::GRIPPER_ROT_REV]))
        {
            _goalJointsPos[(uint8_t)eJointIndex::GRIPPER_ROT] =
                _currentJointsPos[(uint8_t)eJointIndex::GRIPPER_ROT] - MAX_VELOCITY_GRIPPER_ROT;
        }
        else if (isPressed(joyMsg->joy_data[KEYBINDING::GRIPPER_ROT_FWD]))
        {
            _goalJointsPos[(uint8_t)eJointIndex::GRIPPER_ROT] =
                _currentJointsPos[(uint8_t)eJointIndex::GRIPPER_ROT] + MAX_VELOCITY_GRIPPER_ROT;
        }
    }
    if (_controlMode == eControlMode::CARTESIAN)
    {
        // // CARTESIAN controls
        // // =========================================================================
        // _xVelocity = joyMsg->joy_data[rover_msgs::msg::Joy::JOYSTICK_LEFT_FRONT] * -1.0f;
        // _yVelocity = joyMsg->joy_data[rover_msgs::msg::Joy::CROSS_LEFT] - joyMsg->joy_data[rover_msgs::msg::Joy::CROSS_RIGHT];
        // _zVelocity = joyMsg->joy_data[rover_msgs::msg::Joy::JOYSTICK_RIGHT_FRONT];
        // _alphaVelocity = joyMsg->joy_data[rover_msgs::msg::Joy::CROSS_UP] - joyMsg->joy_data[rover_msgs::msg::Joy::CROSS_DOWN];
        // _psiVelocity = joyMsg->joy_data[rover_msgs::msg::Joy::B] - joyMsg->joy_data[rover_msgs::msg::Joy::X];

        // _desiredCartesianVelocity = {_xVelocity, _yVelocity, _zVelocity, _alphaVelocity, _psiVelocity};

        // _jacobian = computeJacobian(sJoint(_currentJointPos[JL], _currentJointPos[J0], _currentJointPos[J1], _currentJointPos[J2], _currentJointPos[GRIPPER_TILT]));
        // _inverseJacobian = arma::inv(_jacobian);

        // _computedJointVelocity = _inverseJacobian * _desiredCartesianVelocity;

        // _constrainedJointVelocity = constrainJointVelocity(_computedJointVelocity, _cartesianMaxJointVelocity);

        // _currentJointPos[JL] += _constrainedJointVelocity(0);
        // _currentJointPos[J0] += _constrainedJointVelocity(1);
        // _currentJointPos[J1] += _constrainedJointVelocity(2);
        // _currentJointPos[J2] += _constrainedJointVelocity(3);
        // _currentJointPos[GRIPPER_TILT] += _constrainedJointVelocity(4);

        // if (_gripperPos)
        // {
        //     _currentJointPos[GRIPPER_CLOSE] += _gripperPos * _maxJointVelocity[GRIPPER_CLOSE];
        // }
        // else if (_currentJointPos[GRIPPER_CLOSE] > 0.0f)
        // {
        //     _currentJointPos[GRIPPER_CLOSE] -= 1.0f * _maxJointVelocity[GRIPPER_CLOSE];
        // }

        // #warning TODO: Find its place
        // _gripperRot = joyMsg->joy_data[rover_msgs::msg::Joy::JOYSTICK_RIGHT_SIDE];
    }

    // CMD GRIP
    if (timerDebounce.isDone() && joyMsg->joy_data[KEYBINDING::GRIPPER_CLOSE])
    {
        _goalJointsPos[(uint8_t)eJointIndex::GRIPPER_CLOSE] = _gripperPos == -1.0f ? 1.0f : -1.0f;
        _gripperPos = _goalJointsPos[(uint8_t)eJointIndex::GRIPPER_CLOSE];
    }

    rover_msgs::msg::ArmMsg msg;
    for (uint8_t i = 0; i < (uint8_t)eJointIndex::eLAST; i++)
    {
        msg.data[i] = _goalJointsPos[i];
    }
    _pub_armCmd->publish(msg);
}

void Teleop::CB_currentPos(const rover_msgs::msg::ArmMsg::SharedPtr armCurrentPos)
{
    _currentPosInvalid = false;
    _timer_armHeartbeat.reset();

    for (uint8_t i = 0; i < (uint8_t)eJointIndex::eLAST; i++)
    {
        _currentJointsPos[i] = armCurrentPos->data[i];
    }
    _currentJointsPos[(uint8_t)eJointIndex::GRIPPER_CLOSE] = _gripperPos;
}

void Teleop::CB_watchdog(bool *lostHB_)
{
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5'000, "Arm watchdog has been triggered!");
    *lostHB_ = true;
}

// arma::mat55 Teleop::computeJacobian(const sJoint &state)
// {
//     arma::mat55 J(arma::fill::zeros);

//     float s1 = sin(state.q1);
//     float c1 = cos(state.q1);
//     float s2 = sin(0.5 * PI - state.q2);
//     float c2 = cos(0.5 * PI - state.q2);
//     float s23 = sin(0.5 * PI - state.q2 - state.q3);
//     float c23 = cos(0.5 * PI - state.q2 - state.q3);
//     float s234 = sin(0.5 * PI - state.q2 - state.q3 - state.q4);
//     float c234 = cos(0.5 * PI - state.q2 - state.q3 - state.q4);

//     J(0, 0) = 0.0f;
//     J(0, 1) = J1x * -s1 + J2x * -s1 * c2 + J2z * -s1 * s2 + J3x * -s1 * c23 + J3z * -s1 * s23 - c1 * (J1y + J2y) + J4x * c1 * c234 + J4z * c1 * s234;
//     J(0, 2) = J2x * c1 * -s2 + J2z * c1 * c2 + J3x * c1 * -s23 + J3z * c1 * c23 + J4x * s1 * -s234 + J4z * s1 * c234;
//     J(0, 3) = J3x * c1 * -s23 + J3z * c1 * c23 + J4x * s1 * -s234 + J4z * s1 * c234;
//     J(0, 4) = J4x * s1 * -s234 + J4z * s1 * c234;

//     J(1, 0) = 1.0f;
//     J(1, 1) = J1x * c1 + -s1 * (J1y + J2y) + J2x * c1 * c2 + J2z * c1 * s2 + J3x * c1 * c23 + J3z * c1 * s23 + J4x * c1 * c234 + J4z * c1 * s234;
//     J(1, 2) = J2x * s1 * -s2 + J2z * s1 * c2 + J3x * s1 * -s23 + J3z * s1 * c23 + J4x * s1 * -s234 + J4z * s1 * c234;
//     J(1, 3) = J3x * s1 * -s23 + J3z * s1 * c23 + J4x * s1 * -s234 + J4z * s1 * c234;
//     J(1, 4) = J4x * s1 * -s234 + J4z * s1 * c234;

//     J(2, 0) = 0.0f;
//     J(2, 1) = 0.0f;
//     J(2, 2) = J2z * -s2 + J3z * -s23 + J4z * -s234 - J2x * c2 - J3x * c23 - J4x * c234;
//     J(2, 3) = J3z * -s23 + J4z * -s234 - J3x * c23 - J4x * c234;
//     J(2, 4) = J4z * -s234 - J4x * c234;

//     J(3, 0) = 0.0f;
//     J(3, 1) = 0.0f;
//     J(3, 2) = 1.0f;
//     J(3, 3) = 1.0f;
//     J(3, 4) = 1.0f;

//     J(4, 0) = 0.0f;
//     J(4, 1) = 1.0f;
//     J(4, 2) = 0.0f;
//     J(4, 3) = 0.0f;
//     J(4, 4) = 0.0f;

//     return J;
// }

// arma::vec5 Teleop::constrainJointVelocity(arma::vec5 jointVelocity, arma::vec maxJointVelocity)
// {
//     arma::vec5 constrainedVelocity = jointVelocity;
//     float highestVelocityRatio = 0.0f;

//     for (uint8_t i = 0; i < jointVelocity.n_elem; i++)
//     {
//         float currentVelocityRatio = std::abs(jointVelocity[i]) / maxJointVelocity[i];
//         if (currentVelocityRatio > highestVelocityRatio)
//         {
//             highestVelocityRatio = currentVelocityRatio;
//         }
//     }

//     if (highestVelocityRatio > 1.0f)
//     {
//         constrainedVelocity /= highestVelocityRatio;
//     }

//     return constrainedVelocity;
// }

rover_msgs::msg::ArmMsg Teleop::getZeroMsg(void)
{
    rover_msgs::msg::ArmMsg msg;
    for (uint8_t i = 0; i < (uint8_t)eJointIndex::eLAST; i++)
    {
        msg.data[i] = _currentJointsPos[i];
    }

    _currentJointsPos[(uint8_t)eJointIndex::GRIPPER_CLOSE] = _gripperPos;

    return msg;
}

bool isPressed(float buttonValue_)
{
    return !IN_ERROR(buttonValue_, 0.01, 0.0f);
}
