#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "rover_msgs/msg/arm_msg.hpp"
#include "rover_msgs/msg/joy.hpp"

#include "rovus_lib/timer.hpp"
#include "rovus_lib/macros.h"

#include "arm_configuration.hpp"
#include "keybinding.hpp"

#include <iostream>
#include <map>

#include "Eigen/Dense"

constexpr std::chrono::milliseconds WATCHDOG_TIMEOUT{500};
constexpr uint64_t TOGGLE_DEBOUNCE_TIME_MS = 150ul;
constexpr float JOINT_CONTROL_SPEED_FACTOR = 0.5f;  // Factor of max speed
constexpr uint8_t MAX_RECORDED_POINTS = 3;

bool isPressed(float buttonValue_);

class Teleop : public rclcpp::Node
{
public:
    enum class eJointIndex : uint8_t
    {
        JL = rover_msgs::msg::ArmMsg::JL,
        // J0 = rover_msgs::msg::ArmMsg::J0,
        J1 = rover_msgs::msg::ArmMsg::J1,
        J2 = rover_msgs::msg::ArmMsg::J2,
        GRIPPER_TILT = rover_msgs::msg::ArmMsg::GRIPPER_TILT,
        GRIPPER_ROT = rover_msgs::msg::ArmMsg::GRIPPER_ROT,
        GRIPPER_CLOSE = rover_msgs::msg::ArmMsg::GRIPPER_CLOSE,
        eLAST
    };

    enum class eButtonId : uint8_t
    {
        RECORD = KEYBINDING::RECORD,
        CREATE_PLAN = KEYBINDING::CREATE_PLAN
    };

    enum class eDesiredCartesianVel : uint8_t
    {
        X = 0,
        Y = 1,
        Z = 2,
        ALPA = 3,
        eLAST
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

    Eigen::MatrixXd _jacobian = Eigen::MatrixXd(4, 4);
    Eigen::MatrixXd _inverseJacobian = Eigen::MatrixXd(4, 4);
    Eigen::MatrixXd _rotationMatrix = Eigen::MatrixXd(3, 3);
    Eigen::VectorXd _computedVelocity = Eigen::VectorXd(4);
    Eigen::VectorXd _currentEndEffectorPosition = Eigen::VectorXd(3); 
    std::array<Eigen::VectorXd, 3> _poseArray;
    
    bool _currentPoseFailure = false;
    bool _gripperClose = false;
    bool _gripperCloseLatchFlag = false;

    std::map<eButtonId, bool> _buttonFlags = {
        {eButtonId::RECORD, false},
        {eButtonId::CREATE_PLAN, false}
    };

    uint8_t _pointsRecorded = 0;

    RoverLib::Timer<uint64_t, RoverLib::millis> timerDebounce
        = RoverLib::Timer<uint64_t, RoverLib::millis>(TOGGLE_DEBOUNCE_TIME_MS);
    
    bool isSelected(float buttonValue_, eButtonId buttonId);
    void positionCallback(const rover_msgs::msg::ArmMsg::SharedPtr positionMsg);
    void joyCallback(const rover_msgs::msg::Joy::SharedPtr positionMsg);
    void scaleVelocities(Eigen::VectorXd& jointVelocities);
    void addPoint(Eigen::VectorXd pose);
    // void applyTransform(Eigen::VectorXd currentPose, std::array<Eigen::VectorXd, 3> poseArray);

    // TODO : remove return values for these functions as they modify class elements directly
    Eigen::MatrixXd computeJacobian(const Eigen::VectorXd& currentJointPosition);

    // FOR TESTING PURPOSES ONLY
    void printEndEffectorPosition();

    void watchdog(bool& rLostHeartbeat);
    rover_msgs::msg::ArmMsg getZeroMsg(void);

    eControlMode _controlMode = eControlMode::CARTESIAN;
    Eigen::VectorXd _currentJointsPos = Eigen::VectorXd::Zero((uint8_t)eJointIndex::eLAST);
    Eigen::VectorXd _desiredCartesian = Eigen::VectorXd::Zero((uint8_t)eDesiredCartesianVel::eLAST);
};

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

void Teleop::joyCallback(const rover_msgs::msg::Joy::SharedPtr joyMsg_)
{
    if (!isPressed(joyMsg_->joy_data[KEYBINDING::DEADMAN_SWITCH]))
    {
        _pubArmCmd->publish(this->getZeroMsg());
        return;
    }

    Eigen::VectorXd goalJointsSpeed = Eigen::VectorXd::Zero((uint8_t)eJointIndex::eLAST);
    Eigen::VectorXd desiredCartesian = Eigen::VectorXd::Zero((uint8_t)eDesiredCartesianVel::eLAST);

    if (_controlMode == eControlMode::JOINT)
    {
        // CMD JL
        if (isPressed(joyMsg_->joy_data[KEYBINDING::JL_FWD]))
        {
            goalJointsSpeed((uint8_t)eJointIndex::JL) = ARM_CONFIGURATION::JL::MAX_VELOCITY;
        }
        else if (isPressed(joyMsg_->joy_data[KEYBINDING::JL_REV]))
        {
            goalJointsSpeed((uint8_t)eJointIndex::JL) = -ARM_CONFIGURATION::JL::MAX_VELOCITY;
        }

        // CMD J0
        // if (isPressed(joyMsg_->joy_data[KEYBINDING::J0_FWD]))
        // {
        //     goalJointsSpeed((uint8_t)eJointIndex::J0) = ARM_CONFIGURATION::J0::MAX_VELOCITY;
        // }
        // else if (isPressed(joyMsg_->joy_data[KEYBINDING::J0_REV]))
        // {
        //     goalJointsSpeed((uint8_t)eJointIndex::J0) = -ARM_CONFIGURATION::J0::MAX_VELOCITY;
        // }

        // CMD J1
        goalJointsSpeed((uint8_t)eJointIndex::J1) = joyMsg_->joy_data[KEYBINDING::J1] * ARM_CONFIGURATION::J1::MAX_VELOCITY;

        // CMD J2
        goalJointsSpeed((uint8_t)eJointIndex::J2) = joyMsg_->joy_data[KEYBINDING::J2] * ARM_CONFIGURATION::J2::MAX_VELOCITY;

        // CMD GRIP_TILT
        if (isPressed(joyMsg_->joy_data[KEYBINDING::GRIPPER_TILT_FWD]))
        {
            goalJointsSpeed((uint8_t)eJointIndex::GRIPPER_TILT) = ARM_CONFIGURATION::GRIPPER_TILT::MAX_VELOCITY;
        }
        else if (isPressed(joyMsg_->joy_data[KEYBINDING::GRIPPER_TILT_REV]))
        {
            goalJointsSpeed((uint8_t)eJointIndex::GRIPPER_TILT) = -ARM_CONFIGURATION::GRIPPER_TILT::MAX_VELOCITY;
        }

        // CMD GRIP_ROT
        if (isPressed(joyMsg_->joy_data[KEYBINDING::GRIPPER_ROT_FWD]))
        {
            goalJointsSpeed((uint8_t)eJointIndex::GRIPPER_ROT) = ARM_CONFIGURATION::GRIPPER_ROT::MAX_VELOCITY;
        }
        else if (isPressed(joyMsg_->joy_data[KEYBINDING::GRIPPER_ROT_REV]))
        {
            goalJointsSpeed((uint8_t)eJointIndex::GRIPPER_ROT) = -ARM_CONFIGURATION::GRIPPER_ROT::MAX_VELOCITY;
        }
    }
    else if(_controlMode == eControlMode::CARTESIAN)
    {
        // CMD X
        if (isPressed(joyMsg_->joy_data[KEYBINDING::X_AXIS_RIGHT]))
        {
            desiredCartesian((uint8_t)eDesiredCartesianVel::X) = joyMsg_->joy_data[KEYBINDING::X_AXIS_RIGHT];
        }        
        if (isPressed(joyMsg_->joy_data[KEYBINDING::X_AXIS_LEFT]))
        {
            desiredCartesian((uint8_t)eDesiredCartesianVel::X) = joyMsg_->joy_data[KEYBINDING::X_AXIS_LEFT] * -1.0f;
        }
        
        // CMD Y
        if (isPressed(joyMsg_->joy_data[KEYBINDING::Y_AXIS]))
        {
            desiredCartesian((uint8_t)eDesiredCartesianVel::Y) = joyMsg_->joy_data[KEYBINDING::Y_AXIS];
        }

        // CMD Z
        if (isPressed(joyMsg_->joy_data[KEYBINDING::Z_AXIS_UP]))
        {
            desiredCartesian((uint8_t)eDesiredCartesianVel::Z) = joyMsg_->joy_data[KEYBINDING::Z_AXIS_UP];
        }        
        if (isPressed(joyMsg_->joy_data[KEYBINDING::Z_AXIS_DOWN]))
        {
            desiredCartesian((uint8_t)eDesiredCartesianVel::Z) = joyMsg_->joy_data[KEYBINDING::Z_AXIS_DOWN] * -1.0f;
        }

        // ALPHA
        if (isPressed(joyMsg_->joy_data[KEYBINDING::ALPHA]))
        {
            desiredCartesian((uint8_t)eDesiredCartesianVel::ALPA) = joyMsg_->joy_data[KEYBINDING::ALPHA];
        }
        
        // PSI
        // if (isPressed(joyMsg_->joy_data[KEYBINDING::PSI]))
        // {
        //     desiredCartesian((uint8_t)eDesiredCartesianVel::PSI) = joyMsg_->joy_data[KEYBINDING::PSI];
        // }

        // RECORD END-EFFECTOR POSE
        if (isSelected(joyMsg_->joy_data[KEYBINDING::RECORD], eButtonId::RECORD))
        {
            addPoint(_currentEndEffectorPosition);
        }

        if (isSelected(joyMsg_->joy_data[KEYBINDING::CREATE_PLAN], eButtonId::CREATE_PLAN))
        {
            if(_pointsRecorded != MAX_RECORDED_POINTS)
            {
                RCLCPP_WARN(LOGGER, "Cannot create plan since not enough points have been gathered");
            }
            else
            {
                Eigen::Vector3d vector12 = _poseArray[1] - _poseArray[0];
                Eigen::Vector3d vector13 = _poseArray[2] - _poseArray[0];
                Eigen::Vector3d zAxis = vector12.cross(vector13).normalized();
                Eigen::Vector3d xAxis = vector12.normalized();     
                Eigen::Vector3d yAxis = zAxis.cross(xAxis);

                _rotationMatrix.col(0) = xAxis;
                _rotationMatrix.col(1) = yAxis;
                _rotationMatrix.col(2) = zAxis;

                desiredCartesian.head(3) = _rotationMatrix * desiredCartesian.head(3);
            }
        }

    _inverseJacobian = computeJacobian(_currentJointsPos).completeOrthogonalDecomposition().pseudoInverse();
        _computedVelocity = _inverseJacobian * desiredCartesian;
        scaleVelocities(_computedVelocity);
        
        for (int i = 0; i < _computedVelocity.size(); ++i)
        {
            goalJointsSpeed(i) = _computedVelocity(i);
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
    goalJointsSpeed((uint8_t)eJointIndex::GRIPPER_CLOSE) = _gripperClose;

    rover_msgs::msg::ArmMsg msg;
    for (uint8_t i = 0; i < (uint8_t)eJointIndex::eLAST; i++)
    {
        msg.data[i] = goalJointsSpeed(i);
    }

    _pubArmCmd->publish(msg);
}

void Teleop::positionCallback(const rover_msgs::msg::ArmMsg::SharedPtr positionMsg_)
{
    _lastPositionData = std::chrono::steady_clock::now();
    _currentPoseFailure = false;

    for (uint8_t i = 0; i < (uint8_t)eJointIndex::eLAST; i++)
    {
        _currentJointsPos(i) = positionMsg_->data[i];
    }

    printEndEffectorPosition();
}

void Teleop::addPoint(Eigen::VectorXd pose_)
{
    if(_pointsRecorded == MAX_RECORDED_POINTS)
    {
        RCLCPP_WARN(LOGGER, "No more points can be recorded. Create a plan or clear all points");
    }
    else
    {
        _poseArray[_pointsRecorded] = pose_;
        _pointsRecorded ++;
    
        RCLCPP_INFO(LOGGER, "Point has been added to array");

        for (size_t i = 0; i < _pointsRecorded; ++i)
        {
            RCLCPP_INFO(LOGGER, "Point %zu: [%f, %f, %f]", i,
                        pose_(0) ,pose_(1), pose_(2));
        }
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

Eigen::MatrixXd Teleop::computeJacobian(const Eigen::VectorXd& currentJointPosition)
{
    float q0 = currentJointPosition(0);
    float q1 = currentJointPosition(1);
    float q2 = currentJointPosition(2);
    float q3 = currentJointPosition(3);

    float s1 = sin(q1);
    float c1 = cos(q1);
    float s12 = sin(q1 + q2);
    float c12 = cos(q1 + q2);
    float s123 = sin(q1 + q2 + q3);
    float c123 = cos(q1 + q2 + q3);

    // float s2 = sin(0.5 * PI - q2);
    // float c2 = cos(0.5 * PI - q2);
    // float s23 = sin(0.5 * PI - q2 - q3);
    // float c23 = cos(0.5 * PI - q2 - q3);

    _currentEndEffectorPosition.x() = q0;

    _currentEndEffectorPosition.y() = J1 * s1 + J2 * s12 + J3 * s123;

    _currentEndEffectorPosition.z() = J1 * s1 + J2 * s12 + J3 * s123;

    // printEndEffectorPosition();
    
    _jacobian(0, 0) = 1.0f; // dx/dq0
    _jacobian(0, 1) = 0.0f; // dx/dq1
    _jacobian(0, 2) = 0.0f; // dx/dq2
    _jacobian(0, 3) = 0.0f; // dx/dq3

    _jacobian(1, 0) = J1 * c1 + J2 * c12 + J3 * c123 + J2; // dy/dq0
    _jacobian(1, 1) = J1 * c1 + J2 * c12 + J3 * c123; // dy/dq1
    _jacobian(1, 2) = J2 * c12 + J3 * c123; // dy/dq2
    _jacobian(1, 3) = J3 * c123; // dy/dq3

    _jacobian(2, 0) = 0.0f; // dz/dq0
    _jacobian(2, 1) = J1 * s1 + J2 * s12 + J3 * s123; // dz/dq1
    _jacobian(2, 2) = J2 * s12 + J3 * s123; // dz/dq2
    _jacobian(2, 3) = J3 * s123; // dz/dq3

    _jacobian(3, 0) = 0.0f; //dxalpha/dq0
    _jacobian(3, 1) = 0.0f; //dxalpha/dq0 
    _jacobian(3, 2) = 0.0f; //dxalpha/dq2
    _jacobian(3, 3) = 0.0f; //dxalpha/dq3

    // _jacobian(0, 0) = 0.0f; // dx/dq0
    // _jacobian(0, 1) = J1x * -s1 + J2x * -s1 * c2 + J2z * -s1 * s2 + J3x * -s1 * c23 + J3z * -s1 * s23 - c1 * (J1y + J2y) + J4x * c1 * c234 + J4z * c1 * s234; // ∂x/dq1
    // _jacobian(0, 2) = J2x * c1 * -s2 + J2z * c1 * c2 + J3x * c1 * -s23 + J3z * c1 * c23 + J4x * s1 * -s234 + J4z * s1 * c234; // dx/dq2
    // _jacobian(0, 3) = J3x * c1 * -s23 + J3z * c1 * c23 + J4x * s1 * -s234 + J4z * s1 * c234; // dx/dq3
    // _jacobian(0, 4) = J4x * s1 * -s234 + J4z * s1 * c234; // dx/dq4

    // _jacobian(1, 0) = 1.0f;  // dy/dq0
    // _jacobian(1, 1) = J1x * c1 + -s1 * (J1y + J2y) + J2x * c1 * c2 + J2z * c1 * s2 + J3x * c1 * c23 + J3z * c1 * s23 + J4x * c1 * c234 + J4z * c1 * s234;  // dy/dq1
    // _jacobian(1, 2) = J2x * s1 * -s2 + J2z * s1 * c2 + J3x * s1 * -s23 + J3z * s1 * c23 + J4x * s1 * -s234 + J4z * s1 * c234;  // dy/dq2
    // _jacobian(1, 3) = J3x * s1 * -s23 + J3z * s1 * c23 + J4x * s1 * -s234 + J4z * s1 * c234;  // dy/dq3
    // _jacobian(1, 4) = J4x * s1 * -s234 + J4z * s1 * c234;  // dy/dq4

    // _jacobian(2, 0) = 0.0f; // dz/dq0
    // _jacobian(2, 1) = 0.0f; // ∂z/dq1
    // _jacobian(2, 2) = J2z * -s2 + J3z * -s23 + J4z * -s234 - J2x * c2 - J3x * c23 - J4x * c234; // dz/dq2
    // _jacobian(2, 3) = J3z * -s23 + J4z * -s234 - J3x * c23 - J4x * c234; // dz/dq3
    // _jacobian(2, 4) = J4z * -s234 - J4x * c234; // dz/dq4

    // NULLSPACE
    // _jacobian(3, 0) = 0.0f;
    // _jacobian(3, 1) = 0.0f;
    // _jacobian(3, 2) = 1.0f;
    // _jacobian(3, 3) = 1.0f;
    // _jacobian(3, 4) = 1.0f;

    // _jacobian(4, 0) = 0.0f;
    // _jacobian(4, 1) = 1.0f;
    // _jacobian(4, 2) = 0.0f;
    // _jacobian(4, 3) = 0.0f;
    // _jacobian(4, 4) = 0.0f;

    return _jacobian;
}

rover_msgs::msg::ArmMsg Teleop::getZeroMsg(void)
{
    rover_msgs::msg::ArmMsg msg;
    msg.data[(uint8_t)eJointIndex::GRIPPER_CLOSE] = false;

    return msg;
}

void Teleop::scaleVelocities(Eigen::VectorXd& velocities)
{
    // float velocityRatio = std::max({
    //     abs(static_cast<float>(velocities(0) / ARM_CONFIGURATION::JL::MAX_VELOCITY)),  
    //     abs(static_cast<float>(velocities(1) / ARM_CONFIGURATION::J0::MAX_VELOCITY)),
    //     abs(static_cast<float>(velocities(2) / ARM_CONFIGURATION::J1::MAX_VELOCITY)),
    //     abs(static_cast<float>(velocities(3) / ARM_CONFIGURATION::J2::MAX_VELOCITY)),
    //     abs(static_cast<float>(velocities(4) / ARM_CONFIGURATION::GRIPPER_TILT::MAX_VELOCITY))
    // });


    // if(velocityRatio > 1.0f)
    // {
    //     velocities /= velocityRatio;
    // }
}

void Teleop::printEndEffectorPosition()
{
    RCLCPP_INFO(
        this->get_logger(),
        "End Effector Position - X: %.3f, Y: %.3f, Z: %.3f",
        _currentEndEffectorPosition.x(),
        _currentEndEffectorPosition.y(),
        _currentEndEffectorPosition.z()
    );
}

bool Teleop::isSelected(float buttonValue_, eButtonId buttonId)
{
    if(isPressed(buttonValue_) && !_buttonFlags[buttonId])
    {
        _buttonFlags[buttonId] = true;
        return isPressed(buttonValue_);
    }
    else if(!isPressed(buttonValue_) && _buttonFlags[buttonId])
    {
        _buttonFlags[buttonId] = false;
        return false;
    }
    else
    {
        return false;
    }
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
