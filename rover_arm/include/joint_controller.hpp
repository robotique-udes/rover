#ifndef __JOINT_CONTROLLER_HPP__
#define __JOINT_CONTROLLER_HPP__

#include "robot_controller.hpp"

class JointController : public RobotController
{
  public:
    JointController(std::initializer_list<eJointIndex> joints_):
        RobotController(joints_),
        _currentControlledJoint()
    {
        if (joints_.size() != 0)
        {
            _currentControlledJoint = *joints_.begin();
        }
        else
        {
            _currentControlledJoint = eJointIndex::eLAST;
        }
    }

    std::array<float, TO_UNDERLYING(eJointIndex::eLAST)> setCmd(
        std::array<float, TO_UNDERLYING(JoyController::eJoyInput::eLAST)> inputArray_) override
    {
        std::array<float, TO_UNDERLYING(eJointIndex::eLAST)> jointCommands = {};

        if (!_joyController.isPressed(inputArray_[KEYBINDINGS::EMILE::DEADMAN_SWITCH]))
        {
            return jointCommands;
        }

        if (ARM_CONFIGURATION::JL::ID == _currentControlledJoint)
        {
            if (_joyController.isPressed(inputArray_[KEYBINDINGS::EMILE::JOINT::JL_RIGHT]))
            {
                jointCommands[TO_UNDERLYING(_currentControlledJoint)] = getMaxVelocity(ARM_CONFIGURATION::JL::ID);
            }
            else if (_joyController.isPressed(inputArray_[KEYBINDINGS::EMILE::JOINT::JL_LEFT]))
            {
                jointCommands[TO_UNDERLYING(_currentControlledJoint)] = -1.0F * getMaxVelocity(ARM_CONFIGURATION::JL::ID);
            }
        }
        if (ARM_CONFIGURATION::J1::ID == _currentControlledJoint)
        {
            if (_joyController.isPressed(inputArray_[KEYBINDINGS::EMILE::JOINT::J1]))
            {
                jointCommands[TO_UNDERLYING(_currentControlledJoint)]
                    = inputArray_[KEYBINDINGS::EMILE::JOINT::J1] * getMaxVelocity(ARM_CONFIGURATION::J1::ID);
            }
        }
        if (ARM_CONFIGURATION::J2::ID == _currentControlledJoint)
        {
            if (_joyController.isPressed(inputArray_[KEYBINDINGS::EMILE::JOINT::J2]))
            {
                jointCommands[TO_UNDERLYING(_currentControlledJoint)]
                    = inputArray_[KEYBINDINGS::EMILE::JOINT::J2] * getMaxVelocity(ARM_CONFIGURATION::J2::ID);
            }
        }

        return jointCommands;
    }

    void setControlledJoint(uint8_t command_)
    {
        size_t currentIndex = TO_UNDERLYING(_currentControlledJoint);

        if (command_ == KEYBINDINGS::EMILE::JOINT::JOINT_SELECT_INC)
        {
            if (currentIndex + 1 < TO_UNDERLYING(eJointIndex::eLAST))
            {
                _currentControlledJoint = static_cast<eJointIndex>(currentIndex + 1);
            }
        }
        else if (command_ == KEYBINDINGS::EMILE::JOINT::JOINT_SELECT_DEC)
        {
            if (_currentControlledJoint > eJointIndex::JL)  // JL REPRESENTS THE JOINT AT INDEX 0
            {
                _currentControlledJoint = static_cast<eJointIndex>(currentIndex - 1);
            }
        }
    }

    eJointIndex getControlledJoint(void)
    {
        return _currentControlledJoint;
    }

  private:
    eJointIndex _currentControlledJoint;
};

#endif