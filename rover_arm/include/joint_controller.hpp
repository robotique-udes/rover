#ifndef __JOINT_CONTROLLER_HPP__
#define __JOINT_CONTROLLER_HPP__

#include "arm_configuration.hpp"
#include "keybinding.hpp"
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

        if (_joyManager.isPressed(KEYBINDINGS::JOINT::JL_RIGHT))
        {
            jointCommands[TO_UNDERLYING(eJointIndex::JL)] = getJogVelocity(ARM_CONFIGURATION::JL::ID);
        }
        else if (_joyManager.isPressed(KEYBINDINGS::JOINT::JL_LEFT))
        {
            jointCommands[TO_UNDERLYING(eJointIndex::JL)] = -1.0F * getJogVelocity(ARM_CONFIGURATION::JL::ID);
        }

        if (_joyManager.isPressed(KEYBINDINGS::JOINT::J1))
        {
            jointCommands[TO_UNDERLYING(eJointIndex::J1)]
                = inputArray_[TO_UNDERLYING(KEYBINDINGS::JOINT::J1)] * getJogVelocity(ARM_CONFIGURATION::J1::ID);
        }
        if (_joyManager.isPressed(KEYBINDINGS::JOINT::J2))
        {
            jointCommands[TO_UNDERLYING(eJointIndex::J2)]
                = inputArray_[TO_UNDERLYING(KEYBINDINGS::JOINT::J2)] * -getJogVelocity(ARM_CONFIGURATION::J2::ID);
        }

        if (_joyManager.isPressed(KEYBINDINGS::JOINT::WRIST_UP))
        {
            jointCommands[TO_UNDERLYING(eJointIndex::GRIPPER_TILT)] = getJogVelocity(ARM_CONFIGURATION::GRIPPER_TILT::ID);
        }
        else if (_joyManager.isPressed(KEYBINDINGS::JOINT::WRIST_DOWN))
        {
            jointCommands[TO_UNDERLYING(eJointIndex::GRIPPER_TILT)] = -1.0F * getJogVelocity(ARM_CONFIGURATION::GRIPPER_TILT::ID);
        }

        if (_joyManager.isPressed(KEYBINDINGS::JOINT::WRIST_ROT_LEFT))
        {
            jointCommands[TO_UNDERLYING(eJointIndex::GRIPPER_ROT)] = getJogVelocity(ARM_CONFIGURATION::GRIPPER_ROT::ID);
        }
        else if (_joyManager.isPressed(KEYBINDINGS::JOINT::WRIST_ROT_RIGHT))
        {
            jointCommands[TO_UNDERLYING(eJointIndex::GRIPPER_ROT)] = -1.0F * getJogVelocity(ARM_CONFIGURATION::GRIPPER_ROT::ID);
        }

        if (_joyManager.isPressed(KEYBINDINGS::JOINT::GRIPPER_CLOSE))
        {
            jointCommands[TO_UNDERLYING(eJointIndex::GRIPPER_CLOSE)] = getJogVelocity(ARM_CONFIGURATION::GRIPPER_CLOSE::ID);
        }
        else if (_joyManager.isPressed(KEYBINDINGS::JOINT::GRIPPER_OPEN))
        {
            jointCommands[TO_UNDERLYING(eJointIndex::GRIPPER_CLOSE)] = -1.0F * getJogVelocity(ARM_CONFIGURATION::GRIPPER_CLOSE::ID);
        }

        return jointCommands;
    }
};

#endif
