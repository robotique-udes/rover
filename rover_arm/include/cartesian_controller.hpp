#ifndef __CARTESIAN_CONTROLLER_HPP__
#define __CARTESIAN_CONTROLLER_HPP__

#include "robot_controller.hpp"
#include "Eigen/Dense"

class CartesianController : public RobotController
{
  public:
    enum class eCartesianCoord
    {
        X = 0,
        Y = 1,
        Z = 2,
        eLAST
    };

    enum class eCartesianInput
    {
        X = 0,
        Y = 1,
        Z = 2,
        ALPHA = 3,
        eLAST
    };

    enum class eCartesianQ
    {
        Q0 = 0,
        Q1 = 1,
        Q2 = 2,
        Q3 = 3,
        eLAST,
    };

    CartesianController(JoyManager& joyManager_):
        RobotController(joyManager_)
    {
    }

  private:
    std::array<float, TO_UNDERLYING(eCartesianInput::eLAST)> _desiredCartesian;
    std::array<float, TO_UNDERLYING(eJointIndex::eLAST)> _jointPositions;
    std::array<std::array<float, TO_UNDERLYING(eCartesianCoord::eLAST)>, TO_UNDERLYING(eCartesianCoord::eLAST)> _poseArray;

    uint8_t _pointsRecorded = 0;
    bool _planApplied = false;

  public:
    std::array<float, TO_UNDERLYING(eJointIndex::eLAST)> getJointCmdFromInput(
        std::array<float, TO_UNDERLYING(eJoyInput::eLAST)> inputArray_) override
    {
        std::array<float, TO_UNDERLYING(eJointIndex::eLAST)> jointCommands = {};
        _desiredCartesian = {};

        if (!_joyManager.isPressed(KEYBINDINGS::EMILE::DEADMAN_SWITCH))
        {
            return jointCommands;
        }

        if (_joyManager.isPressed(KEYBINDINGS::EMILE::CARTESIAN::X_AXIS_RIGHT))
        {
            _desiredCartesian[TO_UNDERLYING(eCartesianInput::X)]
                = inputArray_[TO_UNDERLYING(KEYBINDINGS::EMILE::CARTESIAN::X_AXIS_RIGHT)];
        }
        if (_joyManager.isPressed(KEYBINDINGS::EMILE::CARTESIAN::X_AXIS_LEFT))
        {
            _desiredCartesian[TO_UNDERLYING(eCartesianInput::X)]
                = -1.0F * inputArray_[TO_UNDERLYING(KEYBINDINGS::EMILE::CARTESIAN::X_AXIS_LEFT)];
        }

        if (_joyManager.isPressed(KEYBINDINGS::EMILE::CARTESIAN::Y_AXIS))
        {
            _desiredCartesian[TO_UNDERLYING(eCartesianInput::Y)]
                = -1.0F * inputArray_[TO_UNDERLYING(KEYBINDINGS::EMILE::CARTESIAN::Y_AXIS)];
        }

        if (_joyManager.isPressed(KEYBINDINGS::EMILE::CARTESIAN::Z_AXIS))
        {
            _desiredCartesian[TO_UNDERLYING(eCartesianInput::Z)]
                = inputArray_[TO_UNDERLYING(KEYBINDINGS::EMILE::CARTESIAN::Z_AXIS)];
        }

        if (_joyManager.isPressed(KEYBINDINGS::EMILE::CARTESIAN::ACTIVATE_ALPHA))
        {
            if (_joyManager.isPressed(KEYBINDINGS::EMILE::CARTESIAN::ALPHA_POSITIVE))
            {
                _desiredCartesian[TO_UNDERLYING(eCartesianInput::ALPHA)]
                    = inputArray_[TO_UNDERLYING(KEYBINDINGS::EMILE::CARTESIAN::ALPHA_POSITIVE)];
            }
            else if (_joyManager.isPressed(KEYBINDINGS::EMILE::CARTESIAN::ALPHA_NEGATIVE))
            {
                _desiredCartesian[TO_UNDERLYING(eCartesianInput::ALPHA)]
                    = -1.0F * inputArray_[TO_UNDERLYING(KEYBINDINGS::EMILE::CARTESIAN::ALPHA_NEGATIVE)];
            }
        }

        if (_planApplied)
        {
            Eigen::Vector<float, TO_UNDERLYING(eCartesianCoord::eLAST)> vector12, vector13;

            for (int i = 0; i < TO_UNDERLYING(eCartesianCoord::eLAST); ++i)
            {
                vector12(i) = _poseArray[1][i] - _poseArray[0][i];
                vector13(i) = _poseArray[2][i] - _poseArray[0][i];
            }

            Eigen::Vector<float, TO_UNDERLYING(eCartesianCoord::eLAST)> xAxis;
            xAxis << 1.0F, 0.0F, 0.0F;

            Eigen::Vector<float, TO_UNDERLYING(eCartesianCoord::eLAST)> zAxis = vector12.cross(vector13).normalized();
            Eigen::Vector<float, TO_UNDERLYING(eCartesianCoord::eLAST)> yAxis = zAxis.cross(xAxis).normalized();

            zAxis = xAxis.cross(yAxis).normalized();

            Eigen::Matrix<float, TO_UNDERLYING(eCartesianCoord::eLAST), TO_UNDERLYING(eCartesianCoord::eLAST)> rotationMatrix;
            rotationMatrix.col(TO_UNDERLYING(eCartesianCoord::X)) = xAxis;
            rotationMatrix.col(TO_UNDERLYING(eCartesianCoord::Y)) = yAxis;
            rotationMatrix.col(TO_UNDERLYING(eCartesianCoord::Z)) = zAxis;

            Eigen::Map<Eigen::Vector<float, TO_UNDERLYING(eCartesianCoord::eLAST)>> desiredCartesianVec(_desiredCartesian.data());
            desiredCartesianVec = rotationMatrix * desiredCartesianVec;
        }

        Eigen::Map<
            Eigen::Matrix<float, TO_UNDERLYING(eCartesianInput::eLAST), TO_UNDERLYING(eCartesianQ::eLAST), Eigen::RowMajor>>
            jacobian(this->computeJacobian(_jointPositions).data());
        Eigen::Map<Eigen::Vector<float, TO_UNDERLYING(eCartesianInput::eLAST)>> desiredCartesianVector(_desiredCartesian.data());

        Eigen::Matrix<float, TO_UNDERLYING(eCartesianInput::eLAST), TO_UNDERLYING(eCartesianQ::eLAST)> inverseJacobian
            = jacobian.completeOrthogonalDecomposition().pseudoInverse();
        Eigen::Vector<float, TO_UNDERLYING(eCartesianQ::eLAST)> computedVelocity = inverseJacobian * desiredCartesianVector;

        std::array<float, TO_UNDERLYING(eCartesianQ::eLAST)> velocityArray;
        Eigen::Map<Eigen::Vector<float, TO_UNDERLYING(eCartesianQ::eLAST)>>(velocityArray.data()) = computedVelocity;
        std::array<float, TO_UNDERLYING(eCartesianQ::eLAST)> scaledVelocities = this->scaleVelocities(velocityArray);

        jointCommands[TO_UNDERLYING(eJointIndex::JL)] = scaledVelocities[TO_UNDERLYING(eCartesianQ::Q0)];
        jointCommands[TO_UNDERLYING(eJointIndex::J1)] = scaledVelocities[TO_UNDERLYING(eCartesianQ::Q1)];
        jointCommands[TO_UNDERLYING(eJointIndex::J2)] = scaledVelocities[TO_UNDERLYING(eCartesianQ::Q2)];
        jointCommands[TO_UNDERLYING(eJointIndex::GRIPPER_TILT)] = scaledVelocities[TO_UNDERLYING(eCartesianQ::Q3)];

        return jointCommands;
    }

    std::array<float, TO_UNDERLYING(eCartesianInput::eLAST) * TO_UNDERLYING(eCartesianQ::eLAST)> computeJacobian(
        std::array<float, TO_UNDERLYING(eJointIndex::eLAST)> currentJointPosition_)
    {
        // float q0 = currentJointPosition_[TO_UNDERLYING(eJointIndex::JL)]; // Commented out to avoid unsued variable warning
        float q1 = currentJointPosition_[TO_UNDERLYING(eJointIndex::J1)];
        float q2 = currentJointPosition_[TO_UNDERLYING(eJointIndex::J2)];
        float q3 = currentJointPosition_[TO_UNDERLYING(eJointIndex::GRIPPER_TILT)];

        float s1 = sin(q1);
        float c1 = cos(q1);
        float s12 = sin(q1 + q2);
        float c12 = cos(q1 + q2);
        float s123 = sin(q1 + q2 + q3);
        float c123 = cos(q1 + q2 + q3);

        std::array<float, TO_UNDERLYING(eCartesianInput::eLAST) * TO_UNDERLYING(eCartesianQ::eLAST)> _jacobian;

        _jacobian[0] = 1.0F;  // dx/dq0
        _jacobian[1] = 0.0F;  // dx/dq1
        _jacobian[2] = 0.0F;  // dx/dq2
        _jacobian[3] = 0.0F;  // dx/q3

        _jacobian[4] = 0.0F;                                // dy/dq0
        _jacobian[5] = -J1z * c1 - J2z * c12 - J3z * c123;  // dy/dq1
        _jacobian[6] = -J2z * c12 - J3z * c123;             // dy/dq2
        _jacobian[7] = -J3z * c123;                         // dy/dq3

        _jacobian[8] = 0.0F;                                // dz/dq0
        _jacobian[9] = -J1z * s1 - J2z * s12 - J3z * s123;  // dz/dq1
        _jacobian[10] = -J2z * s12 - J3z * s123;            // dz/dq2
        _jacobian[11] = -J3z * s123;                        // dz/dq3

        _jacobian[12] = 0.0F;  // dalpha/dq0
        _jacobian[13] = 1.0F;  // dalpha/dq1
        _jacobian[14] = 1.0F;  // dalpha/dq2
        _jacobian[15] = 1.0F;  // dalpha/dq3

        return _jacobian;
    }

    std::array<float, TO_UNDERLYING(eCartesianQ::eLAST)> scaleVelocities(
        std::array<float, TO_UNDERLYING(eCartesianQ::eLAST)> velocities_)
    {
        float velocityRatio = std::max({
            fabs(velocities_[TO_UNDERLYING(eCartesianQ::Q0)]) / this->getMaxVelocity(ARM_CONFIGURATION::JL::ID),
            fabs(velocities_[TO_UNDERLYING(eCartesianQ::Q1)]) / this->getMaxVelocity(ARM_CONFIGURATION::J1::ID),
            fabs(velocities_[TO_UNDERLYING(eCartesianQ::Q2)]) / this->getMaxVelocity(ARM_CONFIGURATION::J2::ID),
            fabs(velocities_[TO_UNDERLYING(eCartesianQ::Q3)]) / this->getMaxVelocity(ARM_CONFIGURATION::GRIPPER_TILT::ID),
        });

        if (velocityRatio > 1.0F)
        {
            velocities_[TO_UNDERLYING(eCartesianQ::Q0)] /= velocityRatio;
            velocities_[TO_UNDERLYING(eCartesianQ::Q1)] /= velocityRatio;
            velocities_[TO_UNDERLYING(eCartesianQ::Q2)] /= velocityRatio;
            velocities_[TO_UNDERLYING(eCartesianQ::Q3)] /= velocityRatio;
        }

        return velocities_;
    }

    void addPoint(std::array<float, TO_UNDERLYING(eCartesianCoord::eLAST)> pose_)
    {
        _poseArray[_pointsRecorded] = pose_;
        _pointsRecorded++;
    }

    bool applyPlan(void)
    {
        _planApplied = !_planApplied;

        return _planApplied;
    }

    std::array<float, TO_UNDERLYING(eCartesianCoord::eLAST)> getEndEffectorPose(
        std::array<float, TO_UNDERLYING(eJointIndex::eLAST)> currentJointPosition_)
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

        std::array<float, TO_UNDERLYING(eCartesianCoord::eLAST)> _endEffectorPose;

        _endEffectorPose[TO_UNDERLYING(eCartesianCoord::X)] = q0;
        _endEffectorPose[TO_UNDERLYING(eCartesianCoord::Y)] = J1z * s1 + J2z * s12 + J3z * s123;
        _endEffectorPose[TO_UNDERLYING(eCartesianCoord::Z)] = J1z * c1 + J2z * c12 + J3z * c123;

        return _endEffectorPose;
    }

    uint8_t getRecordedPoints(void)
    {
        return _pointsRecorded;
    }

    void getJointPositions(std::array<float, TO_UNDERLYING(eJointIndex::eLAST)> position_)
    {
        _jointPositions = position_;
    }

    std::array<float, TO_UNDERLYING(eCartesianInput::eLAST)> getDesiredCartesian(void)
    {
        return _desiredCartesian;
    }
};

#endif