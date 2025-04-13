#ifndef __GRIPPER_CONTROLLER_HPP__
#define __GRIPPER_CONTROLLER_HPP__

#include "robot_controller.hpp"

class GripperController : public RobotController
{
  public:
    GripperController(std::initializer_list<eJointIndex> joints_):
        RobotController(joints_)
    {
    }

    std::array<float, TO_UNDERLYING(eJointIndex::eLAST)> setCmd(std::array<float, TO_UNDERLYING(JoyController::eJoyInput::eLAST)> inputArray_) override
    {
        std::array<float, TO_UNDERLYING(eJointIndex::eLAST)> jointCommands = {};

        if (!_joyController.isPressed(inputArray_[KEYBINDINGS::EMILE::DEADMAN_SWITCH]))
        {
            return jointCommands;
        }

        if (_joyController.isPressed(inputArray_[KEYBINDINGS::EMILE::GRIPPER::ROT_FWD]))
        {
            _currentControlledJoint = ARM_CONFIGURATION::GRIPPER_ROT::ID;
            jointCommands[TO_UNDERLYING(_currentControlledJoint)] = getMaxVelocity(ARM_CONFIGURATION::GRIPPER_ROT::ID);
        }
        else if (_joyController.isPressed(inputArray_[KEYBINDINGS::EMILE::GRIPPER::ROT_REV]))
        {
            _currentControlledJoint = ARM_CONFIGURATION::GRIPPER_ROT::ID;
            jointCommands[TO_UNDERLYING(_currentControlledJoint)] = -1.0F * getMaxVelocity(ARM_CONFIGURATION::GRIPPER_ROT::ID);
        }

        if (_joyController.isPressed(inputArray_[KEYBINDINGS::EMILE::GRIPPER::TILT_FWD]))
        {
            _currentControlledJoint = ARM_CONFIGURATION::GRIPPER_TILT::ID;
            jointCommands[TO_UNDERLYING(_currentControlledJoint)] = getMaxVelocity(ARM_CONFIGURATION::GRIPPER_TILT::ID);
        }
        else if (_joyController.isPressed(inputArray_[KEYBINDINGS::EMILE::GRIPPER::TILT_REV]))
        {
            _currentControlledJoint = ARM_CONFIGURATION::GRIPPER_TILT::ID;
            jointCommands[TO_UNDERLYING(_currentControlledJoint)] = -1.0F * getMaxVelocity(ARM_CONFIGURATION::GRIPPER_TILT::ID);
        }

        // VELOCITY CONTROL FOR GRIPPER_CLOSE IS NOT YET IMPLEMENTED
        // if (_joyController.isPressed(inputArray_[KEYBINDINGS::EMILE::GRIPPER::CLOSE]))
        // {
        //     _currentControlledJoint = ARM_CONFIGURATION::GRIPPER_CLOSE::ID;
        //     jointCommands[_currentControlledJoint] = getMaxVelocity(ARM_CONFIGURATION::GRIPPER_CLOSE::ID);
        // }

        return jointCommands;
    }

    eJointIndex getControlledJoint()
    {
        // Could appear in UI
        return _currentControlledJoint;
    }

  private:
    eJointIndex _currentControlledJoint;
};

#endif