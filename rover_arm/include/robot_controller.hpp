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

static constexpr uint8_t ALL_JOINTS = 6;
static constexpr uint8_t ALL_INPUTS = 20;

class RobotController
{
  public:
    virtual ~RobotController() = default;
    RobotController(std::initializer_list<uint8_t> joints_)
    {
        _nJoints = joints_.size();
        _joints.assign(joints_.begin(), joints_.end());
    }

    virtual std::array<float, ALL_JOINTS> setCmd(std::array<float, ALL_INPUTS> inputArray_) = 0;

    static float getMaxVelocity(uint8_t joint)
    {
        switch (joint)
        {
            case TO_UNDERLYING(eJointIndex::JL):
                return ARM_CONFIGURATION::JL::MAX_VELOCITY;
            case TO_UNDERLYING(eJointIndex::J1):
                return ARM_CONFIGURATION::J1::MAX_VELOCITY;
            case TO_UNDERLYING(eJointIndex::J2):
                return ARM_CONFIGURATION::J2::MAX_VELOCITY;
            case TO_UNDERLYING(eJointIndex::GRIPPER_TILT):
                return ARM_CONFIGURATION::GRIPPER_TILT::MAX_VELOCITY;
            case TO_UNDERLYING(eJointIndex::GRIPPER_ROT):
                return ARM_CONFIGURATION::GRIPPER_ROT::MAX_VELOCITY;
            default:
                return 0.0f;
        }
    }

    uint8_t getNJoints(void)
    {
        return _nJoints;
    }

  protected:
    uint8_t _nJoints;
    std::vector<uint8_t> _joints;
    JoyController _joyController;
};

#endif
