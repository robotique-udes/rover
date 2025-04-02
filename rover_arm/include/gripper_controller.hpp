#ifndef __GRIPPER_CONTROLLER_HPP__
#define __GRIPPER_CONTROLLER_HPP__

#include "robot_controller.hpp"

class GripperController : public RobotController
{
  public:

    GripperController(std::initializer_list<uint8_t> joints_):
        RobotController(joints_)
    {
    }

    std::array<float, ALL_JOINTS> setCmd(std::array<float, ALL_INPUTS> inputArray_) override
    {
        std::array<float, ALL_JOINTS> jointCommands = {};

        if (!this->isPressed(inputArray_[KEYBINDINGS_EMILE::DEADMAN_SWITCH]))
        {
            return jointCommands;
        }

        if (this->isPressed(inputArray_[KEYBINDINGS_EMILE::GRIPPER::ROT_FWD]))
        {
            _currentControlledJoint = KEYBINDINGS_EMILE::GRIPPER::ROT_ID;
            jointCommands[_currentControlledJoint] = getMaxVelocity(KEYBINDINGS_EMILE::GRIPPER::ROT_ID);
        }
        else if (this->isPressed(inputArray_[KEYBINDINGS_EMILE::GRIPPER::ROT_REV]))
        {
            _currentControlledJoint = KEYBINDINGS_EMILE::GRIPPER::ROT_ID;
            jointCommands[_currentControlledJoint] = -1.0F * getMaxVelocity(KEYBINDINGS_EMILE::GRIPPER::ROT_ID);
        }

        if (this->isPressed(inputArray_[KEYBINDINGS_EMILE::GRIPPER::TILT_FWD]))
        {
            _currentControlledJoint = KEYBINDINGS_EMILE::GRIPPER::TILT_ID;
            jointCommands[_currentControlledJoint] = getMaxVelocity(KEYBINDINGS_EMILE::GRIPPER::TILT_ID);
        }
        else if (this->isPressed(inputArray_[KEYBINDINGS_EMILE::GRIPPER::TILT_REV]))
        {
            _currentControlledJoint = KEYBINDINGS_EMILE::GRIPPER::TILT_ID;
            jointCommands[_currentControlledJoint] = -1.0F * getMaxVelocity(KEYBINDINGS_EMILE::GRIPPER::TILT_ID);
        }

        // VELOCITY CONTROL FOR GRIPPER_CLOSE IS NOT YET IMPLEMENTED
        if (this->isPressed(inputArray_[KEYBINDINGS_EMILE::GRIPPER::CLOSE]))
        {
            _currentControlledJoint = KEYBINDINGS_EMILE::GRIPPER::CLOSE_ID;
            jointCommands[_currentControlledJoint] = getMaxVelocity(KEYBINDINGS_EMILE::GRIPPER::CLOSE_ID);
        }

        return jointCommands;
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