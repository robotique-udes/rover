#ifndef __ROBOT_CONTROLLER_HPP__
#define __ROBOT_CONTROLLER_HPP__

#include "keybinding.hpp"
#include "joy_controller.hpp"

#include "rovus_lib/timer.hpp"
#include "rovus_lib/macros.h"

#include <stdint.h>
#include <vector>
#include <map>
#include <initializer_list>

class RobotController
{
  public:
    virtual ~RobotController() = default;
    RobotController(std::initializer_list<eJointIndex> joints_):
        _nJoints(joints_.size()),
        _joints(joints_),
        _joyController()
    {
    }

    virtual std::array<float, TO_UNDERLYING(eJointIndex::eLAST)> getJointCmdFromInput(
        std::array<float, TO_UNDERLYING(eJoyInput::eLAST)> inputArray_)
        = 0;

    constexpr float getMaxVelocity(eJointIndex joint_)
    {
        switch (joint_)
        {
            case eJointIndex::JL:
                return ARM_CONFIGURATION::JL::MAX_VELOCITY;
            case eJointIndex::J0:
                return ARM_CONFIGURATION::J0::MAX_VELOCITY;
            case eJointIndex::J1:
                return ARM_CONFIGURATION::J1::MAX_VELOCITY;
            case eJointIndex::J2:
                return ARM_CONFIGURATION::J2::MAX_VELOCITY;
            case eJointIndex::GRIPPER_TILT:
                return ARM_CONFIGURATION::GRIPPER_TILT::MAX_VELOCITY;
            case eJointIndex::GRIPPER_ROT:
                return ARM_CONFIGURATION::GRIPPER_ROT::MAX_VELOCITY;
        }
    }

    uint8_t getNJoints(void)
    {
        return _nJoints;
    }

  protected:
    uint8_t _nJoints;
    std::vector<eJointIndex> _joints;
    JoyController _joyController;
};

#endif
