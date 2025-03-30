#ifndef __GRIPPER_CONTROLLER_HPP__
#define __GRIPPER_CONTROLLER_HPP__

#include "robot_controller.hpp"

class GripperController : public RobotController
{
  public:
    GripperController(std::initializer_list<uint8_t> joints_):
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

        if (KEYBINDINGS_EMILE::GRIPPER::ROT_ID == _currentControlledJoint)
        {
            if (this->isPressed(inputArray_[KEYBINDINGS_EMILE::GRIPPER::ROT_FWD]))
            {
                _currentControlledJoint = KEYBINDINGS_EMILE::GRIPPER::ROT_ID;
                return getMaxVelocity(KEYBINDINGS_EMILE::GRIPPER::ROT_ID);
            }
            else if (this->isPressed(inputArray_[KEYBINDINGS_EMILE::GRIPPER::ROT_REV]))
            {
                _currentControlledJoint = KEYBINDINGS_EMILE::GRIPPER::ROT_ID;
                return -1.0F * getMaxVelocity(KEYBINDINGS_EMILE::GRIPPER::ROT_ID);
            }
        }
        if (KEYBINDINGS_EMILE::GRIPPER::TILT_ID == _currentControlledJoint)
        {
            if (this->isPressed(inputArray_[KEYBINDINGS_EMILE::GRIPPER::TILT_FWD]))
            {
                _currentControlledJoint = KEYBINDINGS_EMILE::GRIPPER::TILT_ID;
                return getMaxVelocity(KEYBINDINGS_EMILE::GRIPPER::TILT_ID);
            }
            else if (this->isPressed(inputArray_[KEYBINDINGS_EMILE::GRIPPER::TILT_REV]))
            {
                _currentControlledJoint = KEYBINDINGS_EMILE::GRIPPER::TILT_ID;
                return -1.0F * getMaxVelocity(KEYBINDINGS_EMILE::GRIPPER::TILT_ID);
            }
        }
        if (KEYBINDINGS_EMILE::GRIPPER::CLOSE_ID == _currentControlledJoint)
        {
            if (this->isPressed(inputArray_[KEYBINDINGS_EMILE::GRIPPER::CLOSE]))
            {
                _currentControlledJoint = KEYBINDINGS_EMILE::GRIPPER::CLOSE_ID;
                return getMaxVelocity(KEYBINDINGS_EMILE::GRIPPER::CLOSE);
            }
        }

        return 0.0F;
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