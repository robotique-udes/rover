#ifndef __JOINT_CONTROLLER_HPP__
#define __JOINT_CONTROLLER_HPP__

#include "robot_controller.hpp"

class JointController : public RobotController
{
  public:
    JointController(std::initializer_list<uint8_t> joints_):
        RobotController(joints_)
    {
        if (joints_.size() != 0)
        {
            _currentControlledJoint = *joints_.begin();
        }
        else
        {
            _currentControlledJoint = -1;
        }
    }

    std::array<float, ALL_JOINTS> setCmd(std::array<float, ALL_INPUTS> inputArray_) override
    {
        std::array<float, ALL_JOINTS> jointCommands = {};

        if (!this->isPressed(inputArray_[KEYBINDINGS_EMILE::DEADMAN_SWITCH]))
        {
            return jointCommands;
        }

        if (KEYBINDINGS_EMILE::JOINT::JL_ID == _currentControlledJoint)
        {
            if (this->isPressed(inputArray_[KEYBINDINGS_EMILE::JOINT::JL_RIGHT]))
            {
                jointCommands[_currentControlledJoint] = getMaxVelocity(KEYBINDINGS_EMILE::JOINT::JL_ID);
            }
            else if (this->isPressed(inputArray_[KEYBINDINGS_EMILE::JOINT::JL_LEFT]))
            {
                jointCommands[_currentControlledJoint] = -1.0F * getMaxVelocity(KEYBINDINGS_EMILE::JOINT::JL_ID);
            }
        }
        if (KEYBINDINGS_EMILE::JOINT::J1_ID == _currentControlledJoint)
        {
            if (this->isPressed(inputArray_[KEYBINDINGS_EMILE::JOINT::J1]))
            {
                jointCommands[_currentControlledJoint] = inputArray_[KEYBINDINGS_EMILE::JOINT::J1] * getMaxVelocity(KEYBINDINGS_EMILE::JOINT::J1_ID);
            }
        }
        if (KEYBINDINGS_EMILE::JOINT::J2_ID == _currentControlledJoint)
        {
            if (this->isPressed(inputArray_[KEYBINDINGS_EMILE::JOINT::J2]))
            {
                jointCommands[_currentControlledJoint] = inputArray_[KEYBINDINGS_EMILE::JOINT::J2] * getMaxVelocity(KEYBINDINGS_EMILE::JOINT::J2_ID);
            }
        }

        return jointCommands;
    }

    void setControlledJoint(uint8_t command_)
    {
        if (command_ == KEYBINDINGS_EMILE::JOINT::JOINT_SELECT_INC)
        {
            if (_currentControlledJoint < _nJoints - 1)
            {
                _currentControlledJoint++;
            }
        }
        else if (command_ == KEYBINDINGS_EMILE::JOINT::JOINT_SELECT_DEC)
        {
            if (_currentControlledJoint > 0)
            {
                _currentControlledJoint--;
            }
        }
    }

    uint8_t getControlledJoint(void)
    {
        return _currentControlledJoint;
    }

  private:
    uint8_t _currentControlledJoint;
};

#endif