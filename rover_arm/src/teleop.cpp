#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/arm_msg.hpp"
#include "rover_msgs/msg/joy.hpp"
#include "rover_msgs/msg/joy_demux_status.hpp"
#include "rover_msgs/msg/propulsion_motor.hpp"
#include "rovus_lib/macros.h"
#include "rovus_lib/timer.hpp"
#include "std_msgs/msg/empty.hpp"

#include "arm_configuration.hpp"
#include "keybinding.hpp"

constexpr uint64_t TOGGLE_DEBOUNCE_TIME_MS = 150ul;
constexpr float JOINT_CONTROL_SPEED_FACTOR = 0.5f;  // Factor of max speed

/// @brief Takes a float from a joy msg and transform it's value into a bool
/// @param buttonValue_
/// @return If interpreted as a click or not
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

    bool _gripperClose = false;
    bool _gripperCloseLatchFlag = false;

    bool _currentPosInvalid = true;
    float _currentJointsPos[(uint8_t)eJointIndex::eLAST] = {0};
    eControlMode _controlMode = eControlMode::JOINT;
    RoverLib::Timer<uint64_t, RoverLib::millis> timerDebounce
        = RoverLib::Timer<uint64_t, RoverLib::millis>(TOGGLE_DEBOUNCE_TIME_MS);

    void CB_joy(const rover_msgs::msg::Joy::SharedPtr joyMsg);
    void CB_currentPos(const rover_msgs::msg::ArmMsg::SharedPtr armCurrentPos);
    void CB_watchdog(bool& rLostHB);
    rover_msgs::msg::ArmMsg getZeroMsg(void);
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Teleop>());
    rclcpp::shutdown();
}

Teleop::Teleop(): Node("teleop")
{
    _timer_armHeartbeat
        = this->create_wall_timer(std::chrono::milliseconds(500), [this]() { this->CB_watchdog(_currentPosInvalid); });

    _sub_joyArm = this->create_subscription<rover_msgs::msg::Joy>("/rover/arm/joy",
                                                                  1,
                                                                  [this](const rover_msgs::msg::Joy::SharedPtr joyMsg_)
                                                                  { this->CB_joy(joyMsg_); });

    _sub_armPosition = this->create_subscription<rover_msgs::msg::ArmMsg>("/rover/arm/status/current_positions",
                                                                          1,
                                                                          [this](const rover_msgs::msg::ArmMsg::SharedPtr msg)
                                                                          { this->CB_currentPos(msg); });

    _pub_armCmd = this->create_publisher<rover_msgs::msg::ArmMsg>("/rover/arm/cmd/goal_speed", 1);
}

void Teleop::CB_joy(const rover_msgs::msg::Joy::SharedPtr joyMsg_)
{
    if (!isPressed(joyMsg_->joy_data[KEYBINDING::DEADMAN_SWITCH]))
    {
        _pub_armCmd->publish(this->getZeroMsg());
        return;
    }

    float _goalJointsSpeed[(uint8_t)eJointIndex::eLAST] = {0};

    if (_controlMode == eControlMode::JOINT)
    {
        // CMD JL
        if (isPressed(joyMsg_->joy_data[KEYBINDING::JL_FWD]))
        {
            _goalJointsSpeed[(uint8_t)eJointIndex::JL] = ARM_CONFIGURATION::JL::MAX_VELOCITY;
        }
        else if (isPressed(joyMsg_->joy_data[KEYBINDING::JL_REV]))
        {
            _goalJointsSpeed[(uint8_t)eJointIndex::JL] = -ARM_CONFIGURATION::JL::MAX_VELOCITY;
        }

        // CMD J0
        if (isPressed(joyMsg_->joy_data[KEYBINDING::J0_FWD]))
        {
            _goalJointsSpeed[(uint8_t)eJointIndex::J0] = ARM_CONFIGURATION::J0::MAX_VELOCITY;
        }
        else if (isPressed(joyMsg_->joy_data[KEYBINDING::J0_REV]))
        {
            _goalJointsSpeed[(uint8_t)eJointIndex::J0] = -ARM_CONFIGURATION::J0::MAX_VELOCITY;
        }

        // CMD J1
        _goalJointsSpeed[(uint8_t)eJointIndex::J1] = joyMsg_->joy_data[KEYBINDING::J1] * ARM_CONFIGURATION::J1::MAX_VELOCITY;
        // CMD J2
        _goalJointsSpeed[(uint8_t)eJointIndex::J2] = joyMsg_->joy_data[KEYBINDING::J2] * ARM_CONFIGURATION::J2::MAX_VELOCITY;

        // CMD GRIP_TILT
        if (isPressed(joyMsg_->joy_data[KEYBINDING::GRIPPER_TILT_FWD]))
        {
            _goalJointsSpeed[(uint8_t)eJointIndex::GRIPPER_TILT] = ARM_CONFIGURATION::GRIPPER_TILT::MAX_VELOCITY;
        }
        else if (isPressed(joyMsg_->joy_data[KEYBINDING::GRIPPER_TILT_REV]))
        {
            _goalJointsSpeed[(uint8_t)eJointIndex::GRIPPER_TILT] = -ARM_CONFIGURATION::GRIPPER_TILT::MAX_VELOCITY;
        }

        // CMD GRIP_ROT
        if (isPressed(joyMsg_->joy_data[KEYBINDING::GRIPPER_ROT_FWD]))
        {
            _goalJointsSpeed[(uint8_t)eJointIndex::GRIPPER_ROT] = ARM_CONFIGURATION::GRIPPER_ROT::MAX_VELOCITY;
        }
        else if (isPressed(joyMsg_->joy_data[KEYBINDING::GRIPPER_ROT_REV]))
        {
            _goalJointsSpeed[(uint8_t)eJointIndex::GRIPPER_ROT] = -ARM_CONFIGURATION::GRIPPER_ROT::MAX_VELOCITY;
        }
    }
    if (_controlMode == eControlMode::CARTESIAN)
    {
        RCLCPP_ERROR(this->get_logger(), "Shouldn't fall here! Not Implemented");
    }

    // CMD GRIP
    if (joyMsg_->joy_data[KEYBINDING::GRIPPER_CLOSE])
    {
        if (timerDebounce.isDone() && !_gripperCloseLatchFlag)
        {
            _gripperClose = !_gripperClose;
            _gripperCloseLatchFlag = true;
        }
    }
    else
    {
        _gripperCloseLatchFlag = false;
    }
    _goalJointsSpeed[(uint8_t)eJointIndex::GRIPPER_CLOSE] = _gripperClose;

    rover_msgs::msg::ArmMsg msg;
    for (uint8_t i = 0; i < (uint8_t)eJointIndex::eLAST; i++)
    {
        msg.data[i] = _goalJointsSpeed[i];
    }
    _pub_armCmd->publish(msg);
}

void Teleop::CB_currentPos(const rover_msgs::msg::ArmMsg::SharedPtr armCurrentPos_)
{
    _timer_armHeartbeat.reset();

    for (uint8_t i = 0; i < (uint8_t)eJointIndex::eLAST; i++)
    {
        _currentJointsPos[i] = armCurrentPos_->data[i];
    }
}

void Teleop::CB_watchdog(bool& rLostHB)
{
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5'000, "Arm watchdog has been triggered!");
    rLostHB = true;
}

rover_msgs::msg::ArmMsg Teleop::getZeroMsg(void)
{
    rover_msgs::msg::ArmMsg msg;
    msg.data[(uint8_t)eJointIndex::GRIPPER_CLOSE] = false;

    return msg;
}

bool isPressed(float buttonValue_)
{
    return !IN_ERROR(buttonValue_, 0.01, 0.0f);
}
