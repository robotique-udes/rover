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

#warning TODO: Timer to make sure it's "alive"
    bool _currentPosInvalid = true;
    float _currentJointsPos[(uint8_t)eJointIndex::eLAST] = {0};
    Teleop::eControlMode _controlMode = Teleop::eControlMode::JOINT;
    Timer<uint64_t, millis> timerDebounce = Timer<uint64_t, millis>(DEBOUNCE_TIME_MS);

    void CB_joy(const rover_msgs::msg::Joy::SharedPtr joyMsg);
    void CB_currentPos(const rover_msgs::msg::ArmMsg::SharedPtr armCurrentPos);
    void CB_watchdog(bool *lostHB_);
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
        RCLCPP_ERROR(this->get_logger(), "Shouldn't fall here!");
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
