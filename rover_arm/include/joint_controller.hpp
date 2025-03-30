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

    float setCmd(std::vector<float> inputArray_) override
    {
        if (!this->isPressed(inputArray_[KEYBINDINGS_EMILE::DEADMAN_SWITCH]))
        {
            return 0.0F;
        }

        if (KEYBINDINGS_EMILE::JOINT::JL::ID == _currentControlledJoint)
        {
            if (this->isPressed(inputArray_[KEYBINDINGS_EMILE::JOINT::JL::JL_RIGHT]))
            {
                return getMaxVelocity(KEYBINDINGS_EMILE::JOINT::JL::ID);
            }
            else if (this->isPressed(inputArray_[KEYBINDINGS_EMILE::JOINT::JL::JL_LEFT]))
            {
                return -1.0F * getMaxVelocity(KEYBINDINGS_EMILE::JOINT::JL::ID);
            }
        }
        if (KEYBINDINGS_EMILE::JOINT::J1::ID == _currentControlledJoint)
        {
            if (this->isPressed(inputArray_[KEYBINDINGS_EMILE::JOINT::J1::J1_FWD]))
            {
                return getMaxVelocity(KEYBINDINGS_EMILE::JOINT::J1::ID);
            }
            else if (this->isPressed(inputArray_[KEYBINDINGS_EMILE::JOINT::J1::J1_REV]))
            {
                return -1.0F * getMaxVelocity(KEYBINDINGS_EMILE::JOINT::J1::ID);
            }
        }
        if (KEYBINDINGS_EMILE::JOINT::J2::ID == _currentControlledJoint)
        {
            if (this->isPressed(inputArray_[KEYBINDINGS_EMILE::JOINT::J2::J2_FWD]))
            {
                return getMaxVelocity(KEYBINDINGS_EMILE::JOINT::J2::ID);
            }
            else if (this->isPressed(inputArray_[KEYBINDINGS_EMILE::JOINT::J2::J2_REV]))
            {
                return -1.0F * getMaxVelocity(KEYBINDINGS_EMILE::JOINT::J2::ID);
            }
        }

        return 0.0F;
    }

    void setControlledJoint(uint8_t command_)
    {
        if (command_ == KEYBINDINGS_EMILE::JOINT::JOINT_SELECTION::JOINT_SELECT_INC)
        {
            if (_currentControlledJoint < _nJoints - 1)
            {
                _currentControlledJoint++;
            }
        }
        else if (command_ == KEYBINDINGS_EMILE::JOINT::JOINT_SELECTION::JOINT_SELECT_DEC)
        {
            if (_currentControlledJoint > 0)
            {
                _currentControlledJoint--;
            }
        }
    }

    uint8_t getControlledJoint()
    {
        // Could appear in UI
        return _currentControlledJoint;
    }

  private:
    uint8_t _currentControlledJoint;
};

#endif