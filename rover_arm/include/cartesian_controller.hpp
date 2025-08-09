#ifndef __CARTESIAN_CONTROLLER_HPP__
#define __CARTESIAN_CONTROLLER_HPP__

#include "robot_controller.hpp"
#include "Eigen/Dense"
#include "rover_lib2/helpers/log.hpp"

DEFINE_LOG_NODE(CartesianController, Logger::eNodeState::ON);

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

    explicit CartesianController(JoyManager& joyManager_):
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

        if (!_joyManager.isPressed(KEYBINDINGS::DEADMAN_SWITCH))
        {
            return jointCommands;
        }

        if (_joyManager.isPressed(KEYBINDINGS::CARTESIAN::X_AXIS_RIGHT))
        {
            _desiredCartesian[TO_UNDERLYING(eCartesianInput::X)]
                = inputArray_[TO_UNDERLYING(KEYBINDINGS::CARTESIAN::X_AXIS_RIGHT)];
        }
        if (_joyManager.isPressed(KEYBINDINGS::CARTESIAN::X_AXIS_LEFT))
        {
            _desiredCartesian[TO_UNDERLYING(eCartesianInput::X)]
                = -1.0F * inputArray_[TO_UNDERLYING(KEYBINDINGS::CARTESIAN::X_AXIS_LEFT)];
        }

        if (_joyManager.isPressed(KEYBINDINGS::CARTESIAN::Y_AXIS))
        {
            _desiredCartesian[TO_UNDERLYING(eCartesianInput::Y)]
                = -1.0F * inputArray_[TO_UNDERLYING(KEYBINDINGS::CARTESIAN::Y_AXIS)];
        }

        if (_joyManager.isPressed(KEYBINDINGS::CARTESIAN::Z_AXIS))
        {
            _desiredCartesian[TO_UNDERLYING(eCartesianInput::Z)] = inputArray_[TO_UNDERLYING(KEYBINDINGS::CARTESIAN::Z_AXIS)];
        }

        if (_joyManager.isPressed(KEYBINDINGS::CARTESIAN::ACTIVATE_ALPHA))
        {
            if (_joyManager.isPressed(KEYBINDINGS::CARTESIAN::ALPHA_POSITIVE))
            {
                _desiredCartesian[TO_UNDERLYING(eCartesianInput::ALPHA)]
                    = inputArray_[TO_UNDERLYING(KEYBINDINGS::CARTESIAN::ALPHA_POSITIVE)];
            }
            else if (_joyManager.isPressed(KEYBINDINGS::CARTESIAN::ALPHA_NEGATIVE))
            {
                _desiredCartesian[TO_UNDERLYING(eCartesianInput::ALPHA)]
                    = -1.0F * inputArray_[TO_UNDERLYING(KEYBINDINGS::CARTESIAN::ALPHA_NEGATIVE)];
            }
        }

        if (_joyManager.isPressed(KEYBINDINGS::JOINT::WRIST_ROT_LEFT))
        {
            jointCommands[TO_UNDERLYING(eJointIndex::GRIPPER_ROT)] = getJogVelocity(ARM_CONFIGURATION::GRIPPER_ROT::ID);
        }
        else if (_joyManager.isPressed(KEYBINDINGS::JOINT::WRIST_ROT_RIGHT))
        {
            jointCommands[TO_UNDERLYING(eJointIndex::GRIPPER_ROT)] = -1.0F * getJogVelocity(ARM_CONFIGURATION::GRIPPER_ROT::ID);
        }

        if (_joyManager.isPressed(KEYBINDINGS::JOINT::GRIPPER_CLOSE))
        {
            jointCommands[TO_UNDERLYING(eJointIndex::GRIPPER_CLOSE)] = getJogVelocity(ARM_CONFIGURATION::GRIPPER_CLOSE::ID);
        }
        else if (_joyManager.isPressed(KEYBINDINGS::JOINT::GRIPPER_OPEN))
        {
            jointCommands[TO_UNDERLYING(eJointIndex::GRIPPER_CLOSE)]
                = -1.0F * getJogVelocity(ARM_CONFIGURATION::GRIPPER_CLOSE::ID);
        }

        if (_planApplied)
        {
            Eigen::Vector<float, TO_UNDERLYING(eCartesianCoord::eLAST)> vector12;
            Eigen::Vector<float, TO_UNDERLYING(eCartesianCoord::eLAST)> vector13;

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
        const std::array<float, TO_UNDERLYING(eJointIndex::eLAST)>& currentJointPosition_) const
    {
        // float q0 = currentJointPosition_[TO_UNDERLYING(eJointIndex::JL)];
        float q1 = currentJointPosition_[TO_UNDERLYING(eJointIndex::J1)];
        float q2 = currentJointPosition_[TO_UNDERLYING(eJointIndex::J2)];
        float q3 = currentJointPosition_[TO_UNDERLYING(eJointIndex::GRIPPER_TILT)];

        float s1 = static_cast<float>(sin(q1));
        float c1 = static_cast<float>(cos(q1));
        float s12 = static_cast<float>(sin(q1 + q2));
        float c12 = static_cast<float>(cos(q1 + q2));
        float s123 = static_cast<float>(sin(q1 + q2 + q3));
        float c123 = static_cast<float>(cos(q1 + q2 + q3));

        std::array<float, TO_UNDERLYING(eCartesianInput::eLAST) * TO_UNDERLYING(eCartesianQ::eLAST)> jacobian;

        jacobian[0] = 1.0F;  // dx/dq0
        jacobian[1] = 0.0F;  // dx/dq1
        jacobian[2] = 0.0F;  // dx/dq2
        jacobian[3] = 0.0F;  // dx/q3

        jacobian[4] = 0.0F;                                // dy/dq0
        jacobian[5] = -J1z * c1 - J2z * c12 - J3z * c123;  // dy/dq1
        jacobian[6] = -J2z * c12 - J3z * c123;             // dy/dq2
        jacobian[7] = -J3z * c123;                         // dy/dq3

        jacobian[8] = 0.0F;                                // dz/dq0
        jacobian[9] = -J1z * s1 - J2z * s12 - J3z * s123;  // dz/dq1
        jacobian[10] = -J2z * s12 - J3z * s123;            // dz/dq2
        jacobian[11] = -J3z * s123;                        // dz/dq3

        jacobian[12] = 0.0F;  // dalpha/dq0
        jacobian[13] = 1.0F;  // dalpha/dq1
        jacobian[14] = 1.0F;  // dalpha/dq2
        jacobian[15] = 1.0F;  // dalpha/dq3

        return jacobian;
    }

    std::array<float, TO_UNDERLYING(eCartesianQ::eLAST)> scaleVelocities(
        std::array<float, TO_UNDERLYING(eCartesianQ::eLAST)> velocities_) const
    {
        double velocityRationDouble = std::max({
            fabs(velocities_[TO_UNDERLYING(eCartesianQ::Q0)]) / this->getCartVelocity(ARM_CONFIGURATION::JL::ID),
            fabs(velocities_[TO_UNDERLYING(eCartesianQ::Q1)]) / this->getCartVelocity(ARM_CONFIGURATION::J1::ID),
            fabs(velocities_[TO_UNDERLYING(eCartesianQ::Q2)]) / this->getCartVelocity(ARM_CONFIGURATION::J2::ID),
            fabs(velocities_[TO_UNDERLYING(eCartesianQ::Q3)]) / this->getCartVelocity(ARM_CONFIGURATION::GRIPPER_TILT::ID),
        });
        float velocityRatio = static_cast<float>(velocityRationDouble);

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
        std::array<float, TO_UNDERLYING(eJointIndex::eLAST)> currentJointPosition_) const
    {
        float q0 = currentJointPosition_[TO_UNDERLYING(eJointIndex::JL)];
        float q1 = currentJointPosition_[TO_UNDERLYING(eJointIndex::J1)];
        float q2 = currentJointPosition_[TO_UNDERLYING(eJointIndex::J2)];
        float q3 = currentJointPosition_[TO_UNDERLYING(eJointIndex::GRIPPER_TILT)];

        float s1 = static_cast<float>(sin(q1));
        float c1 = static_cast<float>(cos(q1));
        float s12 = static_cast<float>(sin(q1 + q2));
        float c12 = static_cast<float>(cos(q1 + q2));
        float s123 = static_cast<float>(sin(q1 + q2 + q3));
        float c123 = static_cast<float>(cos(q1 + q2 + q3));

        std::array<float, TO_UNDERLYING(eCartesianCoord::eLAST)> endEffectorPose;

        endEffectorPose[TO_UNDERLYING(eCartesianCoord::X)] = q0;
        endEffectorPose[TO_UNDERLYING(eCartesianCoord::Y)] = J1z * s1 + J2z * s12 + J3z * s123;
        endEffectorPose[TO_UNDERLYING(eCartesianCoord::Z)] = J1z * c1 + J2z * c12 + J3z * c123;

        return endEffectorPose;
    }

    uint8_t getRecordedPoints(void) const
    {
        return _pointsRecorded;
    }

    void setJointPositions(const std::array<float, TO_UNDERLYING(eJointIndex::eLAST)>& position_)
    {
        _jointPositions = position_;
    }

    std::array<float, TO_UNDERLYING(eCartesianInput::eLAST)> getDesiredCartesian(void) const
    {
        return _desiredCartesian;
    }
};

#endif
