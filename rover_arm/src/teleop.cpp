#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "rover_msgs/msg/arm_msg.hpp"
#include "rover_msgs/msg/joy.hpp"

#include "rovus_lib/timer.hpp"
#include "rovus_lib/macros.h"

#include "arm_configuration.hpp"
#include "keybinding.hpp"

#include "Eigen/Dense"

constexpr std::chrono::milliseconds WATCHDOG_TIMEOUT{500};
constexpr uint64_t TOGGLE_DEBOUNCE_TIME_MS = 150ul;
constexpr float JOINT_CONTROL_SPEED_FACTOR = 0.5f;  // Factor of max speed

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

    enum class eJointIndexInverse : uint8_t
    {
        X = rover_msgs::msg::ArmMsg::JL,
        Y = rover_msgs::msg::ArmMsg::J0,
        Z = rover_msgs::msg::ArmMsg::J1,
    };

    enum class eControlMode : uint8_t
    {
        JOINT = 0,
        CARTESIAN = 1
    };
    
    Teleop();
    ~Teleop() {};

private:
    rclcpp::Subscription<rover_msgs::msg::ArmMsg>::SharedPtr _subArmPositions;
    rclcpp::Subscription<rover_msgs::msg::Joy>::SharedPtr _subJoyArm;
    rclcpp::Publisher<rover_msgs::msg::ArmMsg>::SharedPtr _pubArmCmd;
    rclcpp::TimerBase::SharedPtr _armHeartbeatTimer;    

    std::chrono::steady_clock::time_point _lastPositionData;

    Eigen::MatrixXd _jacobian = Eigen::MatrixXd(3, 5);
    Eigen::MatrixXd _jacobianPseudoInverse = Eigen::MatrixXd(3, 5);
    
    bool _currentPoseFailure = false;
    bool _gripperClose = false;
    bool _gripperCloseLatchFlag = false;

    RoverLib::Timer<uint64_t, RoverLib::millis> timerDebounce
        = RoverLib::Timer<uint64_t, RoverLib::millis>(TOGGLE_DEBOUNCE_TIME_MS);
    
    void positionCallback(const rover_msgs::msg::ArmMsg::SharedPtr positionMsg);
    void joyCallback(const rover_msgs::msg::Joy::SharedPtr positionMsg);
    Eigen::MatrixXd computeJacobian(float _currentJointPosition[7]);

    void watchdog(bool& rLostHeartbeat);
    rover_msgs::msg::ArmMsg getZeroMsg(void);

    eControlMode _controlMode = eControlMode::JOINT;
    float _currentJointsPos[(uint8_t)eJointIndex::eLAST] = {0};
};

void Teleop::joyCallback(const rover_msgs::msg::Joy::SharedPtr joyMsg_)
{
    if (!isPressed(joyMsg_->joy_data[KEYBINDING::DEADMAN_SWITCH]))
    {
        _pubArmCmd->publish(this->getZeroMsg());
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
    else if(_controlMode == eControlMode::CARTESIAN)
    {
        RCLCPP_WARN(LOGGER, "Cartesian mode is currently unavailable due to fault in position data");
        
        if(_currentPoseFailure)
        {
            RCLCPP_WARN(LOGGER, "Cartesian mode is currently unavailable due to fault in position data");
        }

        // CMD X
        if (isPressed(joyMsg_->joy_data[KEYBINDING::X_AXIS_CTRL]))
        {
        }
        
        // CMD Y
        else if (isPressed(joyMsg_->joy_data[KEYBINDING::Y_AXIS_CTRL]))
        {
        }

        // CMD Z
        else if (isPressed(joyMsg_->joy_data[KEYBINDING::Z_AXIS_FWD]))
        {
        }
        else if (isPressed(joyMsg_->joy_data[KEYBINDING::Z_AXIS_BKW]))
        {
        }

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

    _pubArmCmd->publish(msg);
}

void Teleop::positionCallback(const rover_msgs::msg::ArmMsg::SharedPtr positionMsg_)
{
    _lastPositionData = std::chrono::steady_clock::now();
    bool _currentPoseFailure = false;

    for (uint8_t i = 0; i < (uint8_t)eJointIndex::eLAST; i++)
    {
        _currentJointsPos[i] = positionMsg_->data[i];
    }
}

void Teleop::watchdog(bool& rLostHeartbeat_)
{
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - _lastPositionData);
        
    if (elapsed > WATCHDOG_TIMEOUT) 
    {
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(), 
            *this->get_clock(), 
            5000,
            "Arm watchdog has been triggered!"
        );
        rLostHeartbeat_ = true;
    }
}

Eigen::MatrixXd Teleop::computeJacobian(float _currentJointPosition[7])
{

    float q0 = _currentJointPosition[0];
    float q1 = _currentJointPosition[1];
    float q2 = _currentJointPosition[2];
    float q3 = _currentJointPosition[3];
    float q4 = _currentJointPosition[4];

    float s1 = sin(q1);
    float c1 = cos(q1);
    float s2 = sin(0.5 * PI - q2);
    float c2 = cos(0.5 * PI - q2);
    float s23 = sin(0.5 * PI - q2 - q3);
    float c23 = cos(0.5 * PI - q2 - q3);
    float s234 = sin(0.5 * PI - q2 - q3 - q4);
    float c234 = cos(0.5 * PI - q2 - q3 - q4);

    _jacobian(0, 0) = 0.0f; // dx/dq0
    _jacobian(0, 1) = J1x * -s1 + J2x * -s1 * c2 + J2z * -s1 * s2 + J3x * -s1 * c23 + J3z * -s1 * s23 - c1 * (J1y + J2y) + J4x * c1 * c234 + J4z * c1 * s234;; // ∂x/dq1
    _jacobian(0, 2) = J2x * c1 * -s2 + J2z * c1 * c2 + J3x * c1 * -s23 + J3z * c1 * c23 + J4x * s1 * -s234 + J4z * s1 * c234; // dx/dq2
    _jacobian(0, 3) = J3x * c1 * -s23 + J3z * c1 * c23 + J4x * s1 * -s234 + J4z * s1 * c234; // dx/dq3
    _jacobian(0, 4) = J4x * s1 * -s234 + J4z * s1 * c234; // dx/dq4

    _jacobian(1, 0) = 1.0f;  // dy/dq0
    _jacobian(1, 1) = J1x * c1 + -s1 * (J1y + J2y) + J2x * c1 * c2 + J2z * c1 * s2 + J3x * c1 * c23 + J3z * c1 * s23 + J4x * c1 * c234 + J4z * c1 * s234;  // dy/dq1
    _jacobian(1, 2) = J2x * s1 * -s2 + J2z * s1 * c2 + J3x * s1 * -s23 + J3z * s1 * c23 + J4x * s1 * -s234 + J4z * s1 * c234;  // dy/dq2
    _jacobian(1, 3) = J3x * s1 * -s23 + J3z * s1 * c23 + J4x * s1 * -s234 + J4z * s1 * c234;  // dy/dq3
    _jacobian(1, 4) = J4x * s1 * -s234 + J4z * s1 * c234;  // dy/dq4

    _jacobian(2, 0) = 0.0f; // dz/dq0
    _jacobian(2, 1) = 0.0f; // ∂z/dq1
    _jacobian(2, 2) = J2z * -s2 + J3z * -s23 + J4z * -s234 - J2x * c2 - J3x * c23 - J4x * c234; // dz/dq2
    _jacobian(2, 3) = J3z * -s23 + J4z * -s234 - J3x * c23 - J4x * c234; // dz/dq3
    _jacobian(2, 4) = J4z * -s234 - J4x * c234; // dz/dq4

    // NULLSPACE

    _jacobian(3, 0) = 0.0f;
    _jacobian(3, 1) = 0.0f;
    _jacobian(3, 2) = 1.0f;
    _jacobian(3, 3) = 1.0f;
    _jacobian(3, 4) = 1.0f;

    _jacobian(4, 0) = 0.0f;
    _jacobian(4, 1) = 1.0f;
    _jacobian(4, 2) = 0.0f;
    _jacobian(4, 3) = 0.0f;
    _jacobian(4, 4) = 0.0f;

    return _jacobian;
}

rover_msgs::msg::ArmMsg Teleop::getZeroMsg(void)
{
    rover_msgs::msg::ArmMsg msg;
    msg.data[(uint8_t)eJointIndex::GRIPPER_CLOSE] = false;

    return msg;
}

Teleop::Teleop() : Node("teleop")
{
    _subArmPositions = this->create_subscription<rover_msgs::msg::ArmMsg>("/rover/arm/status/current_positions",
                                                                1,
                                                                [this](const rover_msgs::msg::ArmMsg::SharedPtr msg)
                                                                { this->positionCallback(msg); });
                                                                
    _subJoyArm = this->create_subscription<rover_msgs::msg::Joy>("/rover/arm/joy",
                                                                1,
                                                                [this](const rover_msgs::msg::Joy::SharedPtr msg)
                                                                { this->joyCallback(msg); });

    _pubArmCmd = this->create_publisher<rover_msgs::msg::ArmMsg>("/rover/arm/cmd/goal_speed", 1);

    _armHeartbeatTimer = this->create_wall_timer(std::chrono::milliseconds(500),
                                                                [this]()
                                                                { this->watchdog(_currentPoseFailure); });

}

bool isPressed(float buttonValue_)
{
    return !IN_ERROR(buttonValue_, 0.01, 0.0f);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Teleop>());
    rclcpp::shutdown();
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Teleop>());
    rclcpp::shutdown();
}
