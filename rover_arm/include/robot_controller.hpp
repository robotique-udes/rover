#ifndef __ROBOT_CONTROLLER_HPP__
#define __ROBOT_CONTROLLER_HPP__

#include "arm_configuration.hpp"
#include "keybinding.hpp"

#include "rovus_lib/timer.hpp"
#include "rovus_lib/macros.h"

#include <stdint.h>
#include <vector>
#include <map>
#include <initializer_list>

class RobotController
{
  public:
    ~RobotController() = default;
    RobotController(std::initializer_list<uint8_t> joints_)
    {
        _nJoints = joints_.size();
    }

    float setCmd(std::vector<float> inputArray_) {}

    bool isPressed(float buttonValue_) 
    {

    }

    bool isSelected() {}

    static float getMaxVelocity(eJointIndex joint)
    {
        switch (joint)
        {
            case eJointIndex::JL:
                return ARM_CONFIGURATION::JL::MAX_VELOCITY;
            case eJointIndex::J1:
                return ARM_CONFIGURATION::J1::MAX_VELOCITY;
            case eJointIndex::J2:
                return ARM_CONFIGURATION::J2::MAX_VELOCITY;
            case eJointIndex::GRIPPER_TILT:
                return ARM_CONFIGURATION::GRIPPER_TILT::MAX_VELOCITY;
            case eJointIndex::GRIPPER_ROT:
                return ARM_CONFIGURATION::GRIPPER_ROT::MAX_VELOCITY;
            default:
                return 0.0f;
        }
    }

  private:
    uint8_t _nJoints;
};

#endif
