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
    virtual ~RobotController() = default;
    RobotController(std::initializer_list<uint8_t> joints_)
    {
        _nJoints = joints_.size();
        _joints.assign(joints_.begin(), joints_.end());
    }

    virtual float setCmd(std::vector<float> inputArray_) = 0;

    bool isPressed(float buttonValue_)
    {
        return !IN_ERROR(buttonValue_, 0.01, 0.0f);
    }

    bool isSelected(float buttonValue_, uint8_t buttonId)
    {
        if (isPressed(buttonValue_) && !_buttonStates[buttonId])
        {
            _buttonStates[buttonId] = true;
            return true;
        }
        else if (!isPressed(buttonValue_) && _buttonStates[buttonId])
        {
            _buttonStates[buttonId] = false;
            return false; 
        }
        return false; 
    }

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

    uint8_t getNJoints()
    {
        return _nJoints;
    }

  protected:
    uint8_t _nJoints;
    std::vector<uint8_t> _joints;
    std::map<uint8_t, bool> _buttonStates = {{rover_msgs::msg::Joy::CROSS_UP, false},
                                             {rover_msgs::msg::Joy::CROSS_DOWN, false},
                                             {rover_msgs::msg::Joy::CROSS_RIGHT, false},
                                             {rover_msgs::msg::Joy::CROSS_LEFT, false},
                                             {rover_msgs::msg::Joy::A, false},
                                             {rover_msgs::msg::Joy::B, false},
                                             {rover_msgs::msg::Joy::X, false},
                                             {rover_msgs::msg::Joy::Y, false}};
};

#endif
