#ifndef __CARTESIAN_CONTROLLER_HPP__
#define __CARTESIAN_CONTROLLER_HPP__

#include "robot_controller.hpp"
#include "Eigen/Dense"

class CartesianController : public RobotController
{
  public:
    enum class eCartesianR
    {
        X = 0,
        Y = 1,
        Z = 2,
        eLAST
    };

    enum class eCartesianQ
    {
        Q0 = 0,
        Q1 = 1,
        Q2 = 2,
        eLAST,
    };

    CartesianController(std::initializer_list<eJointIndex> joints_):
        RobotController(joints_)
    {
    }

  private:
    std::array<float, TO_UNDERLYING(eCartesianR::eLAST)> _desiredCartesian;
    std::array<float, TO_UNDERLYING(eJointIndex::eLAST)> _jointPositions;
    std::array<std::array<float, TO_UNDERLYING(eCartesianQ::eLAST)>, TO_UNDERLYING(eCartesianQ::eLAST)> _poseArray;

    uint8_t _pointsRecorded = 0;
    bool _planApplied = false;

  public:
    std::array<float, TO_UNDERLYING(eJointIndex::eLAST)> setCmd(
        std::array<float, TO_UNDERLYING(eJoyInput::eLAST)> inputArray_) override
    {
        std::array<float, TO_UNDERLYING(eJointIndex::eLAST)> jointCommands = {};
        _desiredCartesian = {};

        if (!_joyController.isPressed(inputArray_[TO_UNDERLYING(KEYBINDINGS::EMILE::DEADMAN_SWITCH)]))
        {
            return jointCommands;
        }

        if (_joyController.isPressed(inputArray_[TO_UNDERLYING(KEYBINDINGS::EMILE::CARTESIAN::X_AXIS_RIGHT)]))
        {
            _desiredCartesian[TO_UNDERLYING(eCartesianR::X)]
                = inputArray_[TO_UNDERLYING(KEYBINDINGS::EMILE::CARTESIAN::X_AXIS_RIGHT)];
        }
        if (_joyController.isPressed(inputArray_[TO_UNDERLYING(KEYBINDINGS::EMILE::CARTESIAN::X_AXIS_LEFT)]))
        {
            _desiredCartesian[TO_UNDERLYING(eCartesianR::X)]
                = -1.0F * inputArray_[TO_UNDERLYING(KEYBINDINGS::EMILE::CARTESIAN::X_AXIS_LEFT)];
        }

        if (_joyController.isPressed(inputArray_[TO_UNDERLYING(KEYBINDINGS::EMILE::CARTESIAN::Y_AXIS)]))
        {
            _desiredCartesian[TO_UNDERLYING(eCartesianR::Y)]
                = -1.0F * inputArray_[TO_UNDERLYING(KEYBINDINGS::EMILE::CARTESIAN::Y_AXIS)];
        }

        if (_joyController.isPressed(inputArray_[TO_UNDERLYING(KEYBINDINGS::EMILE::CARTESIAN::Z_AXIS)]))
        {
            _desiredCartesian[TO_UNDERLYING(eCartesianR::Z)] = inputArray_[TO_UNDERLYING(KEYBINDINGS::EMILE::CARTESIAN::Z_AXIS)];
        }

        if (_planApplied)
        {
            Eigen::Vector<float, TO_UNDERLYING(eCartesianR::eLAST)> vector12, vector13;

            for (int i = 0; i < TO_UNDERLYING(eCartesianR::eLAST); ++i)
            {
                vector12(i) = _poseArray[1][i] - _poseArray[0][i];
                vector13(i) = _poseArray[2][i] - _poseArray[0][i];
            }

            Eigen::Vector<float, TO_UNDERLYING(eCartesianR::eLAST)> xAxis;
            xAxis << 1.0F, 0.0F, 0.0F;

            Eigen::Vector<float, TO_UNDERLYING(eCartesianR::eLAST)> zAxis = vector12.cross(vector13).normalized();
            Eigen::Vector<float, TO_UNDERLYING(eCartesianR::eLAST)> yAxis = zAxis.cross(xAxis).normalized();

            zAxis = xAxis.cross(yAxis).normalized();

            Eigen::Matrix<float, TO_UNDERLYING(eCartesianR::eLAST), TO_UNDERLYING(eCartesianR::eLAST)> rotationMatrix;
            rotationMatrix.col(0) = xAxis;
            rotationMatrix.col(1) = yAxis;
            rotationMatrix.col(2) = zAxis;

            Eigen::Map<Eigen::Vector<float, TO_UNDERLYING(eCartesianR::eLAST)>> desiredCartesianVec(_desiredCartesian.data());
            desiredCartesianVec = rotationMatrix * desiredCartesianVec;
        }

        Eigen::Map<Eigen::Matrix<float, TO_UNDERLYING(eCartesianR::eLAST), TO_UNDERLYING(eCartesianQ::eLAST), Eigen::RowMajor>>
            jacobian(this->computeJacobian(_jointPositions).data());
        Eigen::Map<Eigen::Vector<float, TO_UNDERLYING(eCartesianR::eLAST)>> desiredCartesianVector(_desiredCartesian.data());

        Eigen::Matrix<float, TO_UNDERLYING(eCartesianR::eLAST), TO_UNDERLYING(eCartesianQ::eLAST)> inverseJacobian
            = jacobian.completeOrthogonalDecomposition().pseudoInverse();
        Eigen::Vector<float, TO_UNDERLYING(eCartesianQ::eLAST)> computedVelocity = inverseJacobian * desiredCartesianVector;

        std::array<float, TO_UNDERLYING(eCartesianQ::eLAST)> velocityArray;
        Eigen::Map<Eigen::Vector<float, TO_UNDERLYING(eCartesianQ::eLAST)>>(velocityArray.data()) = computedVelocity;
        std::array<float, TO_UNDERLYING(eCartesianR::eLAST)> scaledVelocities = this->scaleVelocities(velocityArray);

        jointCommands[TO_UNDERLYING(eJointIndex::JL)] = velocityArray[TO_UNDERLYING(eCartesianQ::Q0)];
        jointCommands[TO_UNDERLYING(eJointIndex::J1)] = velocityArray[TO_UNDERLYING(eCartesianQ::Q1)];
        jointCommands[TO_UNDERLYING(eJointIndex::J2)] = velocityArray[TO_UNDERLYING(eCartesianQ::Q2)];

        return jointCommands;
    }

    std::array<float, TO_UNDERLYING(eCartesianR::eLAST) * TO_UNDERLYING(eCartesianQ::eLAST)> computeJacobian(
        std::array<float, TO_UNDERLYING(eJointIndex::eLAST)> currentJointPosition_)
    {
        // float q0 = currentJointPosition_[TO_UNDERLYING(eJointIndex::JL)]; // Commented out to avoid unsued variable warning
        float q1 = currentJointPosition_[TO_UNDERLYING(eJointIndex::J1)];
        float q2 = currentJointPosition_[TO_UNDERLYING(eJointIndex::J2)];

        float s1 = sin(q1);
        float c1 = cos(q1);
        float s12 = sin(q1 + q2);
        float c12 = cos(q1 + q2);

        uint8_t CARTESIAN_X = TO_UNDERLYING(eCartesianR::X);
        uint8_t CARTESIAN_Y = TO_UNDERLYING(eCartesianR::Y);
        uint8_t CARTESIAN_Z = TO_UNDERLYING(eCartesianR::Z);

        uint8_t CARTESIAN_Q0 = TO_UNDERLYING(eCartesianQ::Q0);
        uint8_t CARTESIAN_Q1 = TO_UNDERLYING(eCartesianQ::Q1);
        uint8_t CARTESIAN_Q2 = TO_UNDERLYING(eCartesianQ::Q2);

        std::array<float, TO_UNDERLYING(eCartesianR::eLAST) * TO_UNDERLYING(eCartesianQ::eLAST)> _jacobian;

        _jacobian[CARTESIAN_X * CARTESIAN_Q0] = 1.0F;  // dx/dq0
        _jacobian[CARTESIAN_X * CARTESIAN_Q1] = 0.0F;  // dx/dq1
        _jacobian[CARTESIAN_X * CARTESIAN_Q2] = 0.0F;  // dx/dq2

        _jacobian[CARTESIAN_Y * CARTESIAN_Q0] = 0.0F;                   // dy/dq0
        _jacobian[CARTESIAN_Y * CARTESIAN_Q1] = -J1z * c1 - J2z * c12;  // dy/dq1
        _jacobian[CARTESIAN_Y * CARTESIAN_Q2] = -J2z * c12;             // dy/dq2

        _jacobian[CARTESIAN_Z * CARTESIAN_Q0] = 0.0F;                   // dz/dq0
        _jacobian[CARTESIAN_Z * CARTESIAN_Q1] = -J1z * s1 - J2z * s12;  // dz/dq1
        _jacobian[CARTESIAN_Z * CARTESIAN_Q2] = -J2z * s12;             // dz/dq2

        return _jacobian;
    }

    std::array<float, TO_UNDERLYING(eCartesianQ::eLAST)> scaleVelocities(
        std::array<float, TO_UNDERLYING(eCartesianQ::eLAST)> velocities_)
    {
        float velocityRatio
            = std::max({fabs(velocities_[TO_UNDERLYING(eCartesianQ::Q0)]) / this->getMaxVelocity(ARM_CONFIGURATION::JL::ID),
                        fabs(velocities_[TO_UNDERLYING(eCartesianQ::Q1)]) / this->getMaxVelocity(ARM_CONFIGURATION::J1::ID),
                        fabs(velocities_[TO_UNDERLYING(eCartesianQ::Q2)]) / this->getMaxVelocity(ARM_CONFIGURATION::J2::ID)});

        if (velocityRatio > 1.0F)
        {
            velocities_[TO_UNDERLYING(eCartesianQ::Q0)] /= velocityRatio;
            velocities_[TO_UNDERLYING(eCartesianQ::Q1)] /= velocityRatio;
            velocities_[TO_UNDERLYING(eCartesianQ::Q2)] /= velocityRatio;
        }

        return velocities_;
    }

    void addPoint(std::array<float, TO_UNDERLYING(eCartesianR::eLAST)> pose_)
    {
        _poseArray[_pointsRecorded] = pose_;
        _pointsRecorded++;
    }

    bool applyPlan(void)
    {
        _planApplied = !_planApplied;

        return _planApplied;
    }

    std::array<float, TO_UNDERLYING(eCartesianR::eLAST)> getEndEffectorPose(
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

        std::array<float, TO_UNDERLYING(eCartesianR::eLAST)> _endEffectorPose;

        _endEffectorPose[TO_UNDERLYING(eCartesianR::X)] = q0;
        _endEffectorPose[TO_UNDERLYING(eCartesianR::Y)] = J1z * s1 + J2z * s12 + J3z * s123;
        _endEffectorPose[TO_UNDERLYING(eCartesianR::Z)] = J1z * c1 + J2z * c12 + J3z * c123;

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

    std::array<float, TO_UNDERLYING(eCartesianR::eLAST)> getDesiredCartesian()
    {
        return _desiredCartesian;
    }
};

#endif