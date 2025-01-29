#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "rover_msgs/msg/arm_msg.hpp"
#include "rover_msgs/msg/joy.hpp"

#include "rovus_lib/timer.hpp"
#include "rovus_lib/macros.h"

#include "arm_configuration.hpp"
#include "keybinding.hpp"

#include "Eigen/Dense"

constexpr uint64_t TOGGLE_DEBOUNCE_TIME_MS = 150ul;
constexpr float JOINT_CONTROL_SPEED_FACTOR = 0.5f;  // Factor of max speed
constexpr uint8_t MAX_RECORDED_POINTS = 3;

// This is the threshold below which singularity avoidance activates. 
// It defines the "danger zone" for approaching singularities.
constexpr float MANIPULABILITY_THRESHOLD = 0.2f;

// This determines how aggressively the robot responds to avoid singularities. 
// It scales the nullspace component added to joint velocities.
constexpr float NULLSPACE_GAIN = 0.4f;

bool isPressed(float buttonValue_);

class Teleop : public rclcpp::Node
{
public:
    enum class eJointIndex : uint8_t
    {
        JL = rover_msgs::msg::ArmMsg::JL,
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
        CLEAR_POINTS = KEYBINDING::CLEAR_POINTS,
        CREATE_PLAN = KEYBINDING::CREATE_PLAN
    };

    enum class eCartesian : uint8_t
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

    enum class eJointIndexInverse : uint8_t
    {
        X = rover_msgs::msg::ArmMsg::JL,
        Y = rover_msgs::msg::ArmMsg::J0,
        Z = rover_msgs::msg::ArmMsg::J1,
    };

    struct sJointVelocity
    {
        float jlVelocity;
        float j0Velocity;
        float j1Velocity;
        float j2Velocity;
        float gripperVelocity;
    };    
    
    // struct sJointPosition
    // {
        // float jlVelocity;
        // float j0Velocity;
        // float j1Velocity;
        // float j2Velocity;
        // float gripperVelocity;
    // };

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
    Eigen::MatrixXd _nullspaceProjector = Eigen::MatrixXd(4, 4);
    Eigen::VectorXd _computedVelocity = Eigen::VectorXd(4);
    Eigen::VectorXd _currentEndEffectorPosition = Eigen::VectorXd(3);
    Eigen::VectorXd _currentJointsPos = Eigen::VectorXd::Zero((uint8_t)eJointIndex::eLAST);
    Eigen::VectorXd _desiredCartesian = Eigen::VectorXd::Zero((uint8_t)eCartesian::eLAST);
    Eigen::VectorXd _gradientManipulability = Eigen::VectorXd(4);
    std::array<Eigen::VectorXd, 3> _poseArray;

    eControlMode _controlMode = eControlMode::CARTESIAN;
    
    uint8_t _pointsRecorded = 0;
    float _manipulabilityMeasure = 0.0f;

    bool _currentPoseFailure = false;
    bool _applyPlan = false;
    bool _gripperClose = false;
    bool _gripperCloseLatchFlag = false;

    Eigen::MatrixXd _jacobian = Eigen::MatrixXd(3, 5);

    bool _currentPosInvalid = false;
    float _currentJointsPos[(uint8_t)eJointIndex::eLAST] = {0};
    eControlMode _controlMode = eControlMode::JOINT;
    RoverLib::Timer<uint64_t, RoverLib::millis> timerDebounce
        = RoverLib::Timer<uint64_t, RoverLib::millis>(TOGGLE_DEBOUNCE_TIME_MS);
    
    bool isSelected(float buttonValue_, eButtonId buttonId_);
    void position_CB(const rover_msgs::msg::ArmMsg::SharedPtr positionMsg_);
    void joy_CB(const rover_msgs::msg::Joy::SharedPtr positionMsg_);
    void scaleVelocities(Eigen::VectorXd& jointVelocities_);
    void addPoint(Eigen::VectorXd pose_);
    void calcManipulability(void);
    void watchdog(bool& rLostHeartbeat_);
    rover_msgs::msg::ArmMsg getZeroMsg(void);
    Eigen::MatrixXd computeJacobian(const Eigen::VectorXd& currentJointPosition_);

    void CB_joy(const rover_msgs::msg::Joy::SharedPtr joyMsg);
    void CB_currentPos(const rover_msgs::msg::ArmMsg::SharedPtr armCurrentPos);
    void CB_watchdog(bool& rLostHB);

    rover_msgs::msg::ArmMsg getZeroMsg(void);
    
    Eigen::MatrixXd computeJacobian(float _currentJointPos[7]);
};

Teleop::Teleop() : Node("teleop")
{
    _subArmPositions = this->create_subscription<rover_msgs::msg::ArmMsg>("/rover/arm/status/current_positions",
                                                                1,
                                                                [this](const rover_msgs::msg::ArmMsg::SharedPtr msg)
                                                                { this->position_CB(msg); });
                                                                
    _subJoyArm = this->create_subscription<rover_msgs::msg::Joy>("/rover/arm/joy",
                                                                1,
                                                                [this](const rover_msgs::msg::Joy::SharedPtr msg)
                                                                { this->joy_CB(msg); });

    _pubArmCmd = this->create_publisher<rover_msgs::msg::ArmMsg>("/rover/arm/cmd/goal_speed", 1);

    _armHeartbeatTimer = this->create_wall_timer(std::chrono::milliseconds(500),
                                                                [this]()
                                                                { this->watchdog(_currentPoseFailure); });
}

void Teleop::joy_CB(const rover_msgs::msg::Joy::SharedPtr joyMsg_)
{
    if (!isPressed(joyMsg_->joy_data[KEYBINDING::DEADMAN_SWITCH]))
    {
        _pubArmCmd->publish(this->getZeroMsg());
        return;
    }

    Eigen::VectorXd goalJointsSpeed = Eigen::VectorXd::Zero((uint8_t)eJointIndex::eLAST);
    Eigen::VectorXd desiredCartesian = Eigen::VectorXd::Zero((uint8_t)eCartesian::eLAST);

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

    }
    else if(_controlMode == eControlMode::CARTESIAN)
    {
        // CMD X
        if (isPressed(joyMsg_->joy_data[KEYBINDING::X_AXIS_RIGHT]))
        {
            desiredCartesian((uint8_t)eCartesian::X) = joyMsg_->joy_data[KEYBINDING::X_AXIS_RIGHT];
        }        
        if (isPressed(joyMsg_->joy_data[KEYBINDING::X_AXIS_LEFT]))
        {
            desiredCartesian((uint8_t)eCartesian::X) = joyMsg_->joy_data[KEYBINDING::X_AXIS_LEFT] * -1.0f;
        }
        
        // CMD Y
        if (isPressed(joyMsg_->joy_data[KEYBINDING::Y_AXIS]))
        {
            desiredCartesian((uint8_t)eCartesian::Y) = joyMsg_->joy_data[KEYBINDING::Y_AXIS] * -1.0f;
        }

        // CMD Z
        if (isPressed(joyMsg_->joy_data[KEYBINDING::Z_AXIS_UP]))
        {
            desiredCartesian((uint8_t)eCartesian::Z) = joyMsg_->joy_data[KEYBINDING::Z_AXIS_UP];
        }        
        if (isPressed(joyMsg_->joy_data[KEYBINDING::Z_AXIS_DOWN]))
        {
            desiredCartesian((uint8_t)eCartesian::Z) = joyMsg_->joy_data[KEYBINDING::Z_AXIS_DOWN] * -1.0f;
        }

        // ALPHA
        if (isPressed(joyMsg_->joy_data[KEYBINDING::ALPHA]))
        {
            desiredCartesian((uint8_t)eCartesian::ALPA) = joyMsg_->joy_data[KEYBINDING::ALPHA];
        }

        // RECORD END-EFFECTOR POSE
        if (isSelected(joyMsg_->joy_data[KEYBINDING::RECORD], eButtonId::RECORD))
        {
            addPoint(_currentEndEffectorPosition);
        }

        // CLEAR POSE ARRAY
        if (isSelected(joyMsg_->joy_data[KEYBINDING::CLEAR_POINTS], eButtonId::CLEAR_POINTS))
        {
            std::array<Eigen::VectorXd, 3>{};
            _pointsRecorded = 0;
            RCLCPP_WARN(LOGGER, "Point array has been cleared");
        }

        if (isSelected(joyMsg_->joy_data[KEYBINDING::CREATE_PLAN], eButtonId::CREATE_PLAN))
        {
            if(_pointsRecorded != MAX_RECORDED_POINTS)
            {
                RCLCPP_WARN(LOGGER, "Cannot create plan since not enough points have been gathered");
            }
            else
            {
                _applyPlan = !_applyPlan;

                if(_applyPlan)
                {
                    RCLCPP_INFO(LOGGER, "Applying plan");
                }
                else if(!_applyPlan)
                {
                    RCLCPP_INFO(LOGGER, "Unapplying plan");
                }
            }
        }

        if(_applyPlan)
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

        _inverseJacobian = computeJacobian(_currentJointsPos).completeOrthogonalDecomposition().pseudoInverse();

        calcManipulability();
        _nullspaceProjector = Eigen::MatrixXd::Identity(4, 4) - _inverseJacobian * _jacobian;

        _computedVelocity = _inverseJacobian * desiredCartesian;

        if (_manipulabilityMeasure < MANIPULABILITY_THRESHOLD) 
        {
            Eigen::VectorXd nullspaceComponent = _nullspaceProjector * _gradientManipulability;
            _computedVelocity += NULLSPACE_GAIN * nullspaceComponent;
        }
        
        scaleVelocities(_computedVelocity);
        
        for (int i = 0; i < _computedVelocity.size(); ++i)
        {
            if(std::abs(_computedVelocity(i)) > 1e-4)
            {
                goalJointsSpeed(i) = _computedVelocity(i);
            }
            else
            {
                goalJointsSpeed(i) = 0.0f;
            }
        }
    }

    // CMD GRIP_ROT
    if (isPressed(joyMsg_->joy_data[KEYBINDING::GRIPPER_ROT_FWD]))
    {
        // RCLCPP_WARN(this->get_logger(), "This is currently being implemented");
        _jacobian = this->computeJacobian(_currentJointsPos);

    }
    else if (isPressed(joyMsg_->joy_data[KEYBINDING::GRIPPER_ROT_REV]))
    {
        goalJointsSpeed((uint8_t)eJointIndex::GRIPPER_ROT) = -ARM_CONFIGURATION::GRIPPER_ROT::MAX_VELOCITY;
    }

    // CMD GRIP_close
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

Eigen::MatrixXd Teleop::computeJacobian(float _currentJointPos[7])
{
    _currentJointPos[(uint8_t)eJointIndex::J0];

    _jacobian(0, 0) = 0;
    _jacobian(0, 1) = -1; 
    _jacobian(0, 2) = -1; 
    _jacobian(0, 3) = -1; 
    _jacobian(0, 4) = -1; 

    _jacobian(1, 0) = 0;
    _jacobian(1, 1) = 1; 
    _jacobian(1, 2) = 1; 
    _jacobian(1, 3) = 1; 
    _jacobian(1, 4) = 1; 

    _jacobian(2, 0) = 1;
    _jacobian(2, 1) = 0;
    _jacobian(2, 2) = 1; 
    _jacobian(2, 3) = 1; 
    _jacobian(2, 4) = 1; 
    
    _jacobian(0, 1) = -1; 
    _jacobian(0, 2) = -1; 
    _jacobian(0, 3) = -1; 
    _jacobian(0, 4) = -1; 

    _jacobian(1, 0) = 0;
    _jacobian(1, 1) = 1; 
    _jacobian(1, 2) = 1; 
    _jacobian(1, 3) = 1; 
    _jacobian(1, 4) = 1; 

    _jacobian(2, 0) = 1;
    _jacobian(2, 1) = 0;
    _jacobian(2, 2) = 1; 
    _jacobian(2, 3) = 1; 
    _jacobian(2, 4) = 1; 

    return _jacobian;
}

void Teleop::CB_currentPos(const rover_msgs::msg::ArmMsg::SharedPtr armCurrentPos_)
{
    _lastPositionData = std::chrono::steady_clock::now();
    _currentPoseFailure = false;

    for (uint8_t i = 0; i < (uint8_t)eJointIndex::eLAST; i++)
    {
        _currentJointsPos(i) = positionMsg_->data[i];
    }
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

Eigen::MatrixXd Teleop::computeJacobian(const Eigen::VectorXd& currentJointPosition_)
{
    float q0 = currentJointPosition_(static_cast<int>(eJointIndex::JL));
    float q1 = currentJointPosition_(static_cast<int>(eJointIndex::J1));
    float q2 = currentJointPosition_(static_cast<int>(eJointIndex::J2));
    float q3 = currentJointPosition_(static_cast<int>(eJointIndex::GRIPPER_TILT));

    float s1 = sin(q1);
    float c1 = cos(q1);
    float s12 = sin(q1 + q2);
    float c12 = cos(q1 + q2);
    float s123 = sin(q1 + q2 + q3);
    float c123 = cos(q1 + q2 + q3);

    _currentEndEffectorPosition.x() = q0;
    _currentEndEffectorPosition.y() = J1z * s1 + J2z * s12 + J3z * s123;
    _currentEndEffectorPosition.z() = J1z * s1 + J2z * s12 + J3z * s123;

    _jacobian(0, 0) = 1.0f;                                 // dx/dq0
    _jacobian(0, 1) = 0.0f;                                 // dx/dq1
    _jacobian(0, 2) = 0.0f;                                 // dx/dq2
    _jacobian(0, 3) = 0.0f;                                 // dx/dq3

    _jacobian(1, 0) = 0.0f;                                 // dy/dq0
    _jacobian(1, 1) = -J1z * c1 - J2z * c12 - J3z * c123;   // dy/dq1
    _jacobian(1, 2) = -J2z * c12 - J3z * c123;              // dy/dq2
    _jacobian(1, 3) = -J3z * c123;                          // dy/dq3

    _jacobian(2, 0) = 0.0f;                                 // dz/dq0
    _jacobian(2, 1) = -J1z * s1 - J2z * s12 - J3z * s123;   // dz/dq1
    _jacobian(2, 2) = -J2z * s12 - J3z * s123;              // dz/dq2
    _jacobian(2, 3) = -J3z * s123;                          // dz/dq3

    _jacobian(3, 0) = 0.0f;                                 // dxalpha/dq0
    _jacobian(3, 1) = 0.0f;                                 // dxalpha/dq0 
    _jacobian(3, 2) = 0.0f;                                 // dxalpha/dq2
    _jacobian(3, 3) = 0.0f;                                 // dxalpha/dq3

    return _jacobian;
}

void Teleop::calcManipulability(void)
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(_jacobian, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::VectorXd singularValues = svd.singularValues();

    _manipulabilityMeasure = singularValues.prod();

    const double h = 0.01;

    for (int i = 0; i < static_cast<int>(eCartesian::eLAST); i++) 
    {
        Eigen::VectorXd perturbedJoints = _currentJointsPos;
        perturbedJoints(i) += h;
        
        Eigen::MatrixXd perturbedJacobian = computeJacobian(perturbedJoints);
        Eigen::JacobiSVD<Eigen::MatrixXd> perturbedSvd(perturbedJacobian, Eigen::ComputeFullU | Eigen::ComputeFullV);
        double perturbedManipulability = perturbedSvd.singularValues().prod();
        
        _gradientManipulability(i) = (perturbedManipulability - _manipulabilityMeasure) / h;
    }
    
    if (_gradientManipulability.norm() > 1e-6)
    {
        _gradientManipulability.normalize();
    }

}

rover_msgs::msg::ArmMsg Teleop::getZeroMsg(void)
{
    rover_msgs::msg::ArmMsg msg;
    msg.data[(uint8_t)eJointIndex::GRIPPER_CLOSE] = false;

    return msg;
}

void Teleop::scaleVelocities(Eigen::VectorXd& velocities_)
{
    float velocityRatio = std::max({
        abs(static_cast<float>(velocities_(0) / ARM_CONFIGURATION::JL::MAX_VELOCITY)),  
        abs(static_cast<float>(velocities_(1) / ARM_CONFIGURATION::J1::MAX_VELOCITY)),
        abs(static_cast<float>(velocities_(2) / ARM_CONFIGURATION::J2::MAX_VELOCITY)),
        abs(static_cast<float>(velocities_(3) / ARM_CONFIGURATION::GRIPPER_TILT::MAX_VELOCITY))
    });

    if(velocityRatio > 1.0f)
    {
        velocities_ /= velocityRatio;
    }
}

bool Teleop::isSelected(float buttonValue_, eButtonId buttonId_)
{
    if(isPressed(buttonValue_) && !_buttonFlags[buttonId_])
    {
        _buttonFlags[buttonId_] = true;
        return isPressed(buttonValue_);
    }
    else if(!isPressed(buttonValue_) && _buttonFlags[buttonId_])
    {
        _buttonFlags[buttonId_] = false;
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
