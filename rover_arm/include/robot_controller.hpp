#ifndef __ROBOT_CONTROLLER_HPP__
#define __ROBOT_CONTROLLER_HPP__

#include "arm_configuration.hpp"
#include "keybinding.hpp"
#include "joy_manager.hpp"

#include <rover_lib2/helpers/macros.hpp>
#include <stdint.h>

class RobotController
{
  public:
    virtual ~RobotController() = default;
    explicit RobotController(JoyManager& joyManager_):
        _joyManager(joyManager_)
    {
    }

    virtual std::array<float, TO_UNDERLYING(eJointIndex::eLAST)> getJointCmdFromInput(
        const std::array<float, TO_UNDERLYING(eJoyInput::eLAST)>& inputArray_)
        = 0;

    constexpr float getCartVelocity(eJointIndex joint_) const
    {
        switch (joint_)
        {
            case eJointIndex::JL:
                return ARM_CONFIGURATION::JL::CART_VELOCITY;
            case eJointIndex::J0:
                return ARM_CONFIGURATION::J0::CART_VELOCITY;
            case eJointIndex::J1:
                return ARM_CONFIGURATION::J1::CART_VELOCITY;
            case eJointIndex::J2:
                return ARM_CONFIGURATION::J2::CART_VELOCITY;
            case eJointIndex::GRIPPER_TILT:
                return ARM_CONFIGURATION::GRIPPER_TILT::CART_VELOCITY;
            case eJointIndex::GRIPPER_ROT:
                return ARM_CONFIGURATION::GRIPPER_ROT::CART_VELOCITY;
            default:
                return 0.0F;
        }
    }

    constexpr float getJogVelocity(eJointIndex joint_) const
    {
        switch (joint_)
        {
            case eJointIndex::JL:
                return ARM_CONFIGURATION::JL::JOG_VELOCITY;
            case eJointIndex::J0:
                return ARM_CONFIGURATION::J0::JOG_VELOCITY;
            case eJointIndex::J1:
                return ARM_CONFIGURATION::J1::JOG_VELOCITY;
            case eJointIndex::J2:
                return ARM_CONFIGURATION::J2::JOG_VELOCITY;
            case eJointIndex::GRIPPER_TILT:
                return ARM_CONFIGURATION::GRIPPER_TILT::JOG_VELOCITY;
            case eJointIndex::GRIPPER_ROT:
                return ARM_CONFIGURATION::GRIPPER_ROT::JOG_VELOCITY;
            case eJointIndex::GRIPPER_CLOSE:
                return ARM_CONFIGURATION::GRIPPER_ROT::JOG_VELOCITY;
            default:
                return 0.0F;
        }
    }

    inline static std::array<float, std::to_underlying(eJointIndex::eLAST)> getNullJointCommands()
    {
        return {};
    }

  protected:
    JoyManager& _joyManager;
};

#endif
