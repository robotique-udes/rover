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
            _currentControlledJoint = 12;
        }
    }

    float setCmd(std::vector<float> inputArray) override
    {
        if (!this->isPressed(inputArray[KEYBINDING::DEADMAN_SWITCH]))
        {
            return 0.0F;
        }

        if (this->isSelected(inputArray[KEYBINDING::JOINT_SELECT_INC], KEYBINDING::JOINT_SELECT_INC))
        {
            setControlledJoint(KEYBINDING::JOINT_SELECT_INC);
        }
        else if (this->isSelected(inputArray[KEYBINDING::JOINT_SELECT_DEC], KEYBINDING::JOINT_SELECT_DEC))
        {
            setControlledJoint(KEYBINDING::JOINT_SELECT_DEC);
        }

        return getMaxVelocity(_joints[_currentControlledJoint]);
    }

    void setControlledJoint(uint8_t command_)
    {
        if (command_ == KEYBINDING::JOINT_SELECT_INC)
        {
            if (_currentControlledJoint < _nJoints - 1)
            {
                _currentControlledJoint++;
            }
        }
        else if (command_ == KEYBINDING::JOINT_SELECT_DEC)
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