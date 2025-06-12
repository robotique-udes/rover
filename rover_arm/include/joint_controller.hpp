#ifndef __JOINT_CONTROLLER_HPP__
#define __JOINT_CONTROLLER_HPP__

#include "robot_controller.hpp"

class JointController : public RobotController
{
  public:
    explicit JointController(JoyManager& joyManager_):
        RobotController(joyManager_)
    {
    }

    std::array<float, TO_UNDERLYING(eJointIndex::eLAST)> getJointCmdFromInput(
        std::array<float, TO_UNDERLYING(eJoyInput::eLAST)> inputArray_) override
    {
        std::array<float, TO_UNDERLYING(eJointIndex::eLAST)> jointCommands = {};

        if (!_joyManager.isPressed(KEYBINDINGS::DEADMAN_SWITCH))
        {
            return jointCommands;
        }

        if (ARM_CONFIGURATION::JL::ID == _currentControlledJoint)
        {
            if (_joyManager.isPressed(KEYBINDINGS::JOINT::JL_RIGHT))
            {
                jointCommands[TO_UNDERLYING(_currentControlledJoint)] = getMaxVelocity(ARM_CONFIGURATION::JL::ID);
            }
            else if (_joyManager.isPressed(KEYBINDINGS::JOINT::JL_LEFT))
            {
                jointCommands[TO_UNDERLYING(_currentControlledJoint)] = -1.0F * getMaxVelocity(ARM_CONFIGURATION::JL::ID);
            }
        }
        // Uncomment when implemented
        // if (ARM_CONFIGURATION::J0::ID == _currentControlledJoint)
        // {
        //     if (_joyController.isPressed(inputArray_[TO_UNDERLYING(KEYBINDINGS::JOINT::J0)]))
        //     {
        //         jointCommands[TO_UNDERLYING(_currentControlledJoint)] = getMaxVelocity(ARM_CONFIGURATION::J0::ID);
        //     }
        // }
        if (ARM_CONFIGURATION::J1::ID == _currentControlledJoint && _joyManager.isPressed(KEYBINDINGS::JOINT::J1))
        {
            jointCommands[TO_UNDERLYING(_currentControlledJoint)]
                = inputArray_[TO_UNDERLYING(KEYBINDINGS::JOINT::J1)] * getMaxVelocity(ARM_CONFIGURATION::J1::ID);
        }
        if (ARM_CONFIGURATION::J2::ID == _currentControlledJoint && _joyManager.isPressed(KEYBINDINGS::JOINT::J2))
        {
            jointCommands[TO_UNDERLYING(_currentControlledJoint)]
                = inputArray_[TO_UNDERLYING(KEYBINDINGS::JOINT::J2)] * getMaxVelocity(ARM_CONFIGURATION::J2::ID);
        }

        if (ARM_CONFIGURATION::GRIPPER_TILT::ID == _currentControlledJoint)
        {
            if (_joyManager.isPressed(KEYBINDINGS::JOINT::WRIST_UP))
            {
                jointCommands[TO_UNDERLYING(_currentControlledJoint)] = getMaxVelocity(ARM_CONFIGURATION::GRIPPER_TILT::ID);
            }
            else if (_joyManager.isPressed(KEYBINDINGS::JOINT::WRIST_DOWN))
            {
                jointCommands[TO_UNDERLYING(_currentControlledJoint)]
                    = -1.0F * getMaxVelocity(ARM_CONFIGURATION::GRIPPER_TILT::ID);
            }
        }

        return jointCommands;
    }

    void setControlledJoint(eJoyInput command_)
    {
        size_t currentIndex = TO_UNDERLYING(_currentControlledJoint);

        if (command_ == KEYBINDINGS::JOINT::JOINT_SELECT_INC)
        {
            if (currentIndex + 1 == TO_UNDERLYING(eJointIndex::J0))
            {
                currentIndex++;
            }
            if (currentIndex + 1 < TO_UNDERLYING(eJointIndex::eLAST))
            {
                _currentControlledJoint = static_cast<eJointIndex>(currentIndex + 1);
            }
        }
        else if (command_ == KEYBINDINGS::JOINT::JOINT_SELECT_DEC)
        {
            if (currentIndex - 1 == TO_UNDERLYING(eJointIndex::J0))
            {
                currentIndex--;
            }
            if (_currentControlledJoint > eJointIndex::JL)  // JL REPRESENTS THE JOINT AT INDEX 0
            {
                _currentControlledJoint = static_cast<eJointIndex>(currentIndex - 1);
            }
        }
    }

    eJointIndex getControlledJoint(void) const
    {
        return _currentControlledJoint;
    }

  private:
    eJointIndex _currentControlledJoint = {eJointIndex::JL};
};

#endif
