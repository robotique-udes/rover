#ifndef __CARTESIAN_CONTROLLER_HPP__
#define __CARTESIAN_CONTROLLER_HPP__

#include "robot_controller.hpp"
#include "Eigen/Dense"

static constexpr uint8_t CARTESIAN_JOINTS = 3;
static constexpr uint8_t MAX_RECORDED_POINTS = 3;

class CartesianController : public RobotController
{
  public:
    enum class eCartesian : uint8_t
    {
        X = 0,
        Y = 1,
        Z = 2
    };

    CartesianController(std::initializer_list<uint8_t> joints_):
        RobotController(joints_)
    {
    }

  private:
    std::array<float, CARTESIAN_JOINTS> _desiredCartesian;
    std::array<float, ALL_JOINTS> _jointPositions;
    std::array<std::array<float, MAX_RECORDED_POINTS>, MAX_RECORDED_POINTS> _poseArray;

    uint8_t _pointsRecorded = 0;
    bool _planApplied = false;

  public:
    std::array<float, ALL_JOINTS> setCmd(std::array<float, ALL_INPUTS> inputArray_) override
    {
        std::array<float, ALL_JOINTS> jointCommands = {};
        _desiredCartesian = {};

        if (!this->isPressed(inputArray_[KEYBINDINGS_EMILE::DEADMAN_SWITCH]))
        {
            return jointCommands;
        }

        if (this->isPressed(inputArray_[KEYBINDINGS_EMILE::CARTESIAN::X_AXIS_RIGHT]))
        {
            _desiredCartesian[TO_UNDERLYING(eCartesian::X)] = inputArray_[KEYBINDINGS_EMILE::CARTESIAN::X_AXIS_RIGHT];
        }
        if (this->isPressed(inputArray_[KEYBINDINGS_EMILE::CARTESIAN::X_AXIS_LEFT]))
        {
            _desiredCartesian[TO_UNDERLYING(eCartesian::X)] = -1.0F * inputArray_[KEYBINDINGS_EMILE::CARTESIAN::X_AXIS_LEFT];
        }

        if (this->isPressed(inputArray_[KEYBINDINGS_EMILE::CARTESIAN::Y_AXIS]))
        {
            _desiredCartesian[TO_UNDERLYING(eCartesian::Y)] = inputArray_[KEYBINDINGS_EMILE::CARTESIAN::Y_AXIS];
        }

        if (this->isPressed(inputArray_[KEYBINDINGS_EMILE::CARTESIAN::Z_AXIS_UP]))
        {
            _desiredCartesian[TO_UNDERLYING(eCartesian::Z)] = inputArray_[KEYBINDINGS_EMILE::CARTESIAN::Z_AXIS_UP];
        }
        if (this->isPressed(inputArray_[KEYBINDINGS_EMILE::CARTESIAN::Z_AXIS_DOWN]))
        {
            _desiredCartesian[TO_UNDERLYING(eCartesian::Z)] = -1.0F * inputArray_[KEYBINDINGS_EMILE::CARTESIAN::Z_AXIS_DOWN];
        }

        // if (_planApplied)
        // {
        //     Eigen::Vector<float, CARTESIAN_JOINTS> vector12, vector13;

        //     for (int i = 0; i < 3; ++i)
        //     {
        //         vector12(i) = _poseArray[1][i] - _poseArray[0][i];
        //         vector13(i) = _poseArray[2][i] - _poseArray[0][i];
        //     }

        //     Eigen::Vector<float, CARTESIAN_JOINTS> zAxis = vector12.cross(vector13).normalized();
        //     Eigen::Vector<float, CARTESIAN_JOINTS> xAxis = vector12.normalized();
        //     Eigen::Vector<float, CARTESIAN_JOINTS> yAxis = zAxis.cross(xAxis);

        //     std::array<float, CARTESIAN_JOINTS * CARTESIAN_JOINTS> _rotationMatrix;
        //     _rotationMatrix = {xAxis(0), yAxis(0), zAxis(0), xAxis(1), yAxis(1), zAxis(1), xAxis(2), yAxis(2), zAxis(2)};

        //     Eigen::Map<Eigen::Matrix<float, 3, 3, Eigen::RowMajor>> rotationMat(_rotationMatrix.data());
        //     Eigen::Map<Eigen::Vector<float, 3>> desiredCartesianVec(_desiredCartesian.data());

        //     desiredCartesianVec = rotationMat * desiredCartesianVec;
        // }

        Eigen::Map<Eigen::Matrix<float, CARTESIAN_JOINTS, CARTESIAN_JOINTS, Eigen::RowMajor>> jacobian(
            this->computeJacobian(_jointPositions).data());
        Eigen::Map<Eigen::Vector<float, CARTESIAN_JOINTS>> desiredCartesianVector(_desiredCartesian.data());

        Eigen::Matrix<float, CARTESIAN_JOINTS, CARTESIAN_JOINTS> inverseJacobian
            = jacobian.completeOrthogonalDecomposition().pseudoInverse();
        Eigen::Vector<float, CARTESIAN_JOINTS> computedVelocity = inverseJacobian * desiredCartesianVector;

        std::array<float, CARTESIAN_JOINTS> velocityArray;
        Eigen::Map<Eigen::Vector<float, CARTESIAN_JOINTS>>(velocityArray.data()) = computedVelocity;
        std::array<float, CARTESIAN_JOINTS> scaledVelocities = this->scaleVelocities(velocityArray);

        jointCommands[TO_UNDERLYING(eJointIndex::JL)] = scaledVelocities[TO_UNDERLYING(eJointIndex::JL)];
        jointCommands[TO_UNDERLYING(eJointIndex::J1)] = scaledVelocities[TO_UNDERLYING(eJointIndex::J1)];
        jointCommands[TO_UNDERLYING(eJointIndex::J2)] = scaledVelocities[TO_UNDERLYING(eJointIndex::J2)];

        return jointCommands;
    }

    std::array<float, CARTESIAN_JOINTS * CARTESIAN_JOINTS> computeJacobian(
        std::array<float, ALL_JOINTS> currentJointPosition_)
    {
        // float q0 = currentJointPosition_[TO_UNDERLYING(eJointIndex::JL)]; // Commented out to avoid unsued variable warning
        float q1 = currentJointPosition_[TO_UNDERLYING(eJointIndex::J1)];
        float q2 = currentJointPosition_[TO_UNDERLYING(eJointIndex::J2)];

        float s1 = sin(q1);
        float c1 = cos(q1);
        float s12 = sin(q1 + q2);
        float c12 = cos(q1 + q2);

        std::array<float, CARTESIAN_JOINTS * CARTESIAN_JOINTS> _jacobian;

        _jacobian[0 * CARTESIAN_JOINTS + 0] = 1.0f;  // dx/dq0
        _jacobian[0 * CARTESIAN_JOINTS + 1] = 0.0f;  // dx/dq1
        _jacobian[0 * CARTESIAN_JOINTS + 2] = 0.0f;  // dx/dq2

        _jacobian[1 * CARTESIAN_JOINTS + 0] = 0.0f;                   // dy/dq0
        _jacobian[1 * CARTESIAN_JOINTS + 1] = -J1z * c1 - J2z * c12;  // dy/dq1
        _jacobian[1 * CARTESIAN_JOINTS + 2] = -J2z * c12;             // dy/dq2

        _jacobian[2 * CARTESIAN_JOINTS + 0] = 0.0f;                   // dz/dq0
        _jacobian[2 * CARTESIAN_JOINTS + 1] = -J1z * s1 - J2z * s12;  // dz/dq1
        _jacobian[2 * CARTESIAN_JOINTS + 2] = -J2z * s12;             // dz/dq2

        return _jacobian;
    }

    std::array<float, CARTESIAN_JOINTS> scaleVelocities(std::array<float, CARTESIAN_JOINTS> velocities_)
    {
        float velocityRatio
            = std::max({fabs(velocities_[TO_UNDERLYING(eJointIndex::JL)]) / this->getMaxVelocity(TO_UNDERLYING(eJointIndex::JL)),
                        fabs(velocities_[TO_UNDERLYING(eJointIndex::J1)]) / this->getMaxVelocity(TO_UNDERLYING(eJointIndex::J1)),
                        fabs(velocities_[TO_UNDERLYING(eJointIndex::J2)]) / this->getMaxVelocity(TO_UNDERLYING(eJointIndex::J2))});

        if (velocityRatio > 1.0F)
        {
            velocities_[TO_UNDERLYING(eJointIndex::JL)] /= velocityRatio;
            velocities_[TO_UNDERLYING(eJointIndex::J1)] /= velocityRatio;
            velocities_[TO_UNDERLYING(eJointIndex::J2)] /= velocityRatio;
        }

        return velocities_;
    }

    void addPoint(std::array<float, MAX_RECORDED_POINTS> pose_)
    {
        _poseArray[_pointsRecorded] = pose_;
        _pointsRecorded++;
    }

    bool applyPlan(void)
    {
        _planApplied = !_planApplied;

        return _planApplied;
    }

    std::array<float, CARTESIAN_JOINTS> getEndEffectorPose(std::array<float, ALL_JOINTS> currentJointPosition_)
    {
        float q0 = currentJointPosition_[TO_UNDERLYING(eJointIndex::JL)];
        float q1 = currentJointPosition_[TO_UNDERLYING(eJointIndex::J1)];
        float q2 = currentJointPosition_[TO_UNDERLYING(eJointIndex::J2)];
        float q3 = currentJointPosition_[TO_UNDERLYING(eJointIndex::GRIPPER_TILT)];

        float s1 = sin(q1);
        float c1 = cos(q1);
        float s12 = sin(q1 + q2);
        float c12 = cos(q1 + q2);
        float s123 = sin(q1 + q2 + q3);
        float c123 = cos(q1 + q2 + q3);

        std::array<float, CARTESIAN_JOINTS> _endEffectorPose;

        _endEffectorPose[TO_UNDERLYING(eCartesian::X)] = q0;
        _endEffectorPose[TO_UNDERLYING(eCartesian::Y)] = J1z * s1 + J2z * s12 + J3z * s123;
        _endEffectorPose[TO_UNDERLYING(eCartesian::Z)] = J1z * c1 + J2z * c12 + J3z * c123;

        return _endEffectorPose;
    }

    uint8_t getRecordedPoints(void)
    {
        return _pointsRecorded;
    }

    void getJointPositions(std::array<float, ALL_JOINTS> position_)
    {
        _jointPositions = position_;
    }

    std::array<float, CARTESIAN_JOINTS> getDesiredCartesian()
    {
        return _desiredCartesian;
    }
};

#endif